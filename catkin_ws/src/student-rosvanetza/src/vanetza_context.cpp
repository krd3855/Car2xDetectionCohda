    #include "cmc-ccu.pb.h"
#include "dcc_pass_through.hpp"
#include "radio_configurator.hpp"
#include "vanetza_context.hpp"
#include "vanetza/LinkLayerStamped.h"
#include <boost/range/adaptor/sliced.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <vanetza/access/data_request.hpp>
#include <vanetza/dcc/bursty_transmit_rate_control.hpp>
#include <vanetza/dcc/channel_load.hpp>
#include <vanetza/dcc/flow_control.hpp>
#include <vanetza/dcc/gradual_state_machine.hpp>
#include <vanetza/net/mac_address.hpp>
#include <vanetza/net/cohesive_packet.hpp>
#include <vanetza/units/time.hpp>
#include <ros/console.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <random>

using namespace vanetza;

template<typename REQ>
class TransmissionContext
{
public:
    TransmissionContext(geonet::Router& router, const REQ& request, std::unique_ptr<DownPacket> packet) :
        router_(router), request_(request), packet_(std::move(packet))
    {
    }

    void operator()()
    {
        if (packet_)
        {
            geonet::DataConfirm confirm = router_.request(request_, std::move(packet_));
            confirm_promise_.set_value(confirm);
        }
    }

    std::future<geonet::DataConfirm> get_future()
    {
        return confirm_promise_.get_future();
    }

private:
    geonet::Router& router_;
    REQ request_;
    std::unique_ptr<DownPacket> packet_;
    std::promise<geonet::DataConfirm> confirm_promise_;
};


class AccessInterfacePublisher : public vanetza::access::Interface
{
public:
    AccessInterfacePublisher(const ros::Publisher& pub, std::unique_ptr<vanetza::access::Interface> access) :
        publisher_(pub), access_(std::move(access))
    {
    }

    void request(const vanetza::access::DataRequest& request, std::unique_ptr<vanetza::ChunkPacket> packet) override
    {
        auto msg = boost::make_shared<LinkLayerStamped>();
        msg->header.stamp = ros::Time::now();
        std::copy_n(request.source_addr.octets.data(), request.source_addr.octets.size(), msg->link_layer.source.data());
        std::copy_n(request.destination_addr.octets.data(), request.destination_addr.octets.size(), msg->link_layer.destination.data());
        for (auto& layer : osi_layer_range<OsiLayer::Network, OsiLayer::Application>()) {
            auto byte_view = create_byte_view(packet->layer(layer));
            std::copy(byte_view.begin(), byte_view.end(), std::back_inserter(msg->link_layer.data));
        }
        publisher_.publish(msg);

        access_->request(request, std::move(packet));
    }

private:
    ros::Publisher publisher_;
    std::unique_ptr<vanetza::access::Interface> access_;
};


//TODO remove or cancel
void reasonPacketDrop(vanetza::geonet::Router::PacketDropReason packetDropReason){
    std::string reason = vanetza::geonet::stringify(packetDropReason);
    ROS_INFO_STREAM("Packet dropped due to " << reason);
}

void reasonStopForwarding(vanetza::geonet::Router::ForwardingStopReason forwardingStopReason){
    std::string reason_string;

    // TODO replace this by something more elegant, e.g. https://github.com/aantron/better-enums
    switch (forwardingStopReason) {
        case vanetza::geonet::Router::ForwardingStopReason::Hop_Limit:
            reason_string = "Hop limit";
            break;
        case vanetza::geonet::Router::ForwardingStopReason::Source_PDR:
            reason_string = "Source PDR";
            break;
        case vanetza::geonet::Router::ForwardingStopReason::Sender_PDR:
            reason_string = "Sender PDR";
            break;
        case vanetza::geonet::Router::ForwardingStopReason::Outside_Destination_Area:
            reason_string = "Outside Destination Area";
            break;

        default:
            reason_string = "UNKNOWN";
            break;
    }

    ROS_INFO_STREAM("Packet forwarding stopped due to " << reason_string);
}



VanetzaContext::VanetzaContext(boost::asio::io_service& io, TimeTrigger& trigger, vanetza::PositionProvider& positioning,
        vanetza::geonet::TransportInterface& transport, const ContextParams& params) :
    mIoService(io),
    mTimeTrigger(trigger), mTransport(transport), mPositionProvider(positioning),
    mMib(configureMib(params)), mRouter(mTimeTrigger, mMib), mParams(params)
{
    const auto ccu_ip = boost::asio::ip::address_v4::from_string(mParams.ccuIp);
    const boost::asio::ip::udp::endpoint ccu_endpoint(ccu_ip, mParams.ccuPort);

    mAccessInterface.reset(new ProtobufAccessInterface(mIoService, ccu_endpoint));
    if (mParams.publishLinkLayer) {
        mPublisherRx = mNodeHandle.advertise<LinkLayerStamped>("link_layer_rx", 40);
        mPublisherTx = mNodeHandle.advertise<LinkLayerStamped>("link_layer_tx", 20);
        mAccessInterface = std::unique_ptr<vanetza::access::Interface> {
            new AccessInterfacePublisher(mPublisherTx, std::move(mAccessInterface))
        };
    }

    mMulticastReceiver.reset(new MulticastReceiver(mIoService, boost::asio::ip::address::from_string(mParams.multicastIp), mParams.multicastPort, this));
    mDccStateMachine.reset(new vanetza::dcc::GradualStateMachine(vanetza::dcc::etsiStates1ms));
    mTransmitRateControl.reset(new vanetza::dcc::BurstyTransmitRateControl(*mDccStateMachine, mTimeTrigger));
    if (params.gatekeeper) {
        mDccInterface.reset(new vanetza::dcc::FlowControl(mTimeTrigger, *mTransmitRateControl, *mAccessInterface));
    } else {
        mDccInterface.reset(new DccPassThrough(*mAccessInterface));
    }

    boost::asio::io_service io_service;
    RadioConfigurator radio_config(io_service);
    radio_config.setLinkLayerAddress(mMib.itsGnLocalGnAddr.mid());
    radio_config.setDefaultTxPower(mParams.defaultTxPower);
    radio_config.run(ccu_endpoint);

    mRouter.set_address(mMib.itsGnLocalGnAddr);
    mRouter.set_access_interface(mDccInterface.get());
    mRouter.set_transport_handler(vanetza::geonet::UpperProtocol::BTP_A, &mTransport);
    mRouter.set_transport_handler(vanetza::geonet::UpperProtocol::BTP_B, &mTransport);
    mRouter.packet_dropped = reasonPacketDrop;
    mRouter.forwarding_stopped = reasonStopForwarding;

    updatePositionVector();
}

VanetzaContext::~VanetzaContext()
{
    if (mThread.joinable()) {
        mIoService.stop();
        mThread.join();
    }
}

void VanetzaContext::passMessageToRouter(std::unique_ptr<LinkLayerReception> received)
{
    vanetza::ByteBuffer buf(received->payload().begin(), received->payload().end());
    std::unique_ptr<vanetza::PacketVariant> packet {
        new vanetza::PacketVariant { vanetza::CohesivePacket(std::move(buf), vanetza::OsiLayer::Network) }
    };

    if (received->source().size() != MacAddress::length_bytes) {
        ROS_ERROR("received packet's source MAC address is invalid");
    } else if (received->destination().size() != MacAddress::length_bytes) {
        ROS_ERROR("received packet's destination MAC address is invalid");
    } else {
        MacAddress source;
        std::copy_n(received->source().begin(), MacAddress::length_bytes, source.octets.begin());
        MacAddress destination;
        std::copy_n(received->destination().begin(), MacAddress::length_bytes, destination.octets.begin());

        ROS_DEBUG_STREAM("CCU link layer received by " << mMib.itsGnLocalGnAddr.mid() << " for " << destination << " from " << source);
        mRouter.indicate(std::move(packet), source, destination);

        if (mParams.publishLinkLayer) {
            auto msg = boost::make_shared<LinkLayerStamped>();
            msg->header.stamp = ros::Time::now();
            std::copy_n(source.octets.data(), source.octets.size(), msg->link_layer.source.data());
            std::copy_n(destination.octets.data(), destination.octets.size(), msg->link_layer.destination.data());
            std::copy(received->payload().begin(), received->payload().end(), std::back_inserter(msg->link_layer.data));
            mPublisherRx.publish(msg);
        }
    }
}

void VanetzaContext::passCbrToRouter(const ChannelBusyRatio& cbr)
{
    ROS_DEBUG("CBR measurement received from CCU (%i, %i)", cbr.busy(), cbr.total());
    mDccStateMachine->update(vanetza::dcc::ChannelLoad(cbr.busy(), cbr.total()));
}

std::future<geonet::DataConfirm> VanetzaContext::requestTransmission(const geonet::ShbDataRequest& request, std::unique_ptr<DownPacket> packet)
{
    ROS_DEBUG("SHB transmission requested");
    using Context = TransmissionContext<geonet::ShbDataRequest>;
    auto context = std::make_shared<Context>(mRouter, request, std::move(packet));
    mIoService.post([context]() { (*context)(); });
    return context->get_future();
}

std::future<geonet::DataConfirm> VanetzaContext::requestTransmission(const geonet::GbcDataRequest& request, std::unique_ptr<DownPacket> packet)
{
    ROS_DEBUG("GBC transmission requested (repetitions: %s)", request.repetition ? "yes" : "no");
    using Context = TransmissionContext<geonet::GbcDataRequest>;
    auto context = std::make_shared<Context>(mRouter, request, std::move(packet));
    mIoService.post([context]() { (*context)(); });
    return context->get_future();
}

void VanetzaContext::updatePositionVector()
{
    mRouter.update_position(mPositionProvider.position_fix());

    vanetza::Clock::duration next = std::chrono::milliseconds(100);
    vanetza::Runtime::Callback callback = [this](vanetza::Clock::time_point tp) {
        updatePositionVector();
    };
    mTimeTrigger.schedule(next, callback);
}

vanetza::geonet::MIB VanetzaContext::configureMib(const ContextParams& params)
{
    vanetza::MacAddress mac;
    static_assert(mac.octets.size() == 6, "invalid length of MAC address");
    boost::range::copy(boost::adaptors::slice(params.pseudonym, 2, 8), mac.octets.begin());
    // clear broadcast/multicast address bit
    mac.octets[0] &= ~0x01;
    // set locally unique address bit
    mac.octets[0] |= 0x02;
    ROS_INFO_STREAM("Own MAC address is set to " << mac);

    vanetza::geonet::MIB mib;
    mib.itsGnLocalGnAddr.mid(mac);
    mib.itsGnLocalGnAddr.is_manually_configured(true);
    mib.itsGnLocalGnAddr.station_type(params.stationType);
    mib.itsGnLocalAddrConfMethod = vanetza::geonet::AddrConfMethod::Auto;
    mib.itsGnSecurity = false; // TODO: set up and enable security
    mib.itsGnDefaultTrafficClass.tc_id(3); // send Beacons with DP3
    mib.vanetzaDefaultSeed = std::random_device()();
    mib.vanetzaDeferInitialBeacon = true; // do not immediately send a beacon on start-up

    // reset neighbour flag in location table entries after some time without update
    if (params.softStateNeighbours) {
        using vanetza::units::clock_cast;
        mib.vanetzaNeighbourFlagExpiry = clock_cast(mib.itsGnBeaconServiceRetransmitTimer + mib.itsGnBeaconServiceMaxJitter);
    }

    // keep CBF counters longer than actual contention time to reduce duplicates
    mib.vanetzaFadingCbfCounter = params.fadingCbfCounters;

    return mib;
}

void VanetzaContext::startRouter()
{
    mThread = std::thread([this]() { run(); });
}

//should be started in new thread!
void VanetzaContext::run()
{
    mMulticastReceiver->startReceive();
    try {
        mIoService.run();
    } catch (boost::system::system_error& e) {
        ROS_ERROR_STREAM("IO thread stopped: " << e.what());
        ros::shutdown();
    }
}
