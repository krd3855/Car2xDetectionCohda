#ifndef VANETZACONTEXT_HPP_
#define VANETZACONTEXT_HPP_

#include "multicast_receiver.hpp"
#include "protobuf_access_interface.hpp"
#include "time_trigger.hpp"
#include "vanetza/btp/data_interface.hpp"
#include "vanetza/common/byte_buffer.hpp"
#include "vanetza/common/position_provider.hpp"
#include "vanetza/dcc/interface.hpp"
#include "vanetza/dcc/transmit_rate_control.hpp"
#include "vanetza/dcc/state_machine.hpp"
#include "vanetza/geonet/data_confirm.hpp"
#include "vanetza/geonet/mib.hpp"
#include "vanetza/geonet/router.hpp"
#include "vanetza/geonet/station_type.hpp"
#include "vanetza/geonet/transport_interface.hpp"
#include "vanetza/security/basic_elements.hpp"
#include <boost/asio/io_service.hpp>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <future>
#include <limits>

class ChannelBusyRatio;
class LinkLayerReception;
class MulticastReceiver;

struct ContextParams
{
    std::string multicastIp;
    int multicastPort;
    std::string ccuIp;
    int ccuPort;
    vanetza::security::HashedId8 pseudonym;
    vanetza::geonet::StationType stationType;
    bool gatekeeper = true; /*< DCC gatekeeper at link layer */
    bool softStateNeighbours = false; /*< enable soft-state neighbour state in location table (non standard) */
    bool fadingCbfCounters = false; /* use fading Contention-Based Forwarding counters (non standard) */
    bool publishLinkLayer = false; /* publish link layer messages (rx and tx) */
    double defaultTxPower = std::numeric_limits<double>::quiet_NaN();
};

class VanetzaContext
{
public:
    VanetzaContext(boost::asio::io_service&, TimeTrigger&, vanetza::PositionProvider&, vanetza::geonet::TransportInterface&, const ContextParams&);
    ~VanetzaContext();

    void startRouter();

    void passMessageToRouter(std::unique_ptr<LinkLayerReception>);
    void passCbrToRouter(const ChannelBusyRatio&);
    const vanetza::geonet::MIB getMib() const { return mMib; }
    std::future<vanetza::geonet::DataConfirm> requestTransmission(const vanetza::geonet::ShbDataRequest&, std::unique_ptr<vanetza::DownPacket>);
    std::future<vanetza::geonet::DataConfirm> requestTransmission(const vanetza::geonet::GbcDataRequest&, std::unique_ptr<vanetza::DownPacket>);

private:
    void run();
    void updatePositionVector();

    static vanetza::geonet::MIB configureMib(const ContextParams&);

    std::thread mThread;
    boost::asio::io_service& mIoService;
    TimeTrigger& mTimeTrigger;
    vanetza::PositionProvider& mPositionProvider;
    vanetza::geonet::TransportInterface& mTransport;
    const vanetza::geonet::MIB mMib;
    vanetza::geonet::Router mRouter;
    ContextParams mParams;

    ros::NodeHandle mNodeHandle;
    ros::Publisher mPublisherRx;
    ros::Publisher mPublisherTx;

    std::unique_ptr<vanetza::access::Interface> mAccessInterface;
    std::unique_ptr<vanetza::dcc::RequestInterface> mDccInterface;
    std::unique_ptr<vanetza::dcc::TransmitRateControl> mTransmitRateControl;
    std::unique_ptr<vanetza::dcc::StateMachine> mDccStateMachine;
    std::unique_ptr<MulticastReceiver> mMulticastReceiver;

    };

#endif /* VANETZACONTEXT_HPP_ */
