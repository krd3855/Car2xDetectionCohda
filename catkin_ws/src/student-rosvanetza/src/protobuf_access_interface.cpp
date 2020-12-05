#include "protobuf_access_interface.hpp"
#include "cmc-ccu.pb.h"
#include <vanetza/access/data_request.hpp>
#include <vanetza/common/byte_view.hpp>
#include <vanetza/net/chunk_packet.hpp>
#include <vanetza/net/osi_layer.hpp>
#include <ros/console.h>
#include <memory>
#include <string>

using namespace vanetza;

std::unique_ptr<LinkLayerTransmission> createLinkLayerTx(const vanetza::access::DataRequest& req, std::unique_ptr<vanetza::ChunkPacket> packet)
{
    std::unique_ptr<LinkLayerTransmission> transmission { new LinkLayerTransmission() };
    transmission->set_source(req.source_addr.octets.data(), req.source_addr.octets.size());
    transmission->set_destination(req.destination_addr.octets.data(), req.destination_addr.octets.size());
    LinkLayerPriority prio = LinkLayerPriority::BEST_EFFORT;
    switch (req.access_category) {
        case access::AccessCategory::VO:
            prio = LinkLayerPriority::VOICE;
            break;
        case access::AccessCategory::VI:
            prio = LinkLayerPriority::VIDEO;
            break;
        case access::AccessCategory::BE:
            prio = LinkLayerPriority::BEST_EFFORT;
            break;
        case access::AccessCategory::BK:
            prio = LinkLayerPriority::BACKGROUND;
            break;
        default:
            ROS_WARN("unknown access category requested, falling back to best effort");
            break;
    }
    transmission->set_priority(prio);

    std::string* payload = transmission->mutable_payload();
    for (auto& layer : osi_layer_range<OsiLayer::Network, OsiLayer::Application>()) {
        auto byte_view = create_byte_view(packet->layer(layer));
        payload->append(byte_view.begin(), byte_view.end());
    }

    return transmission;
}

ProtobufAccessInterface::ProtobufAccessInterface(boost::asio::io_service& io, const boost::asio::ip::udp::endpoint& ccu) :
    mSocket(io)
{
    mSocket.connect(ccu);
}

void ProtobufAccessInterface::request(const vanetza::access::DataRequest& req, std::unique_ptr<vanetza::ChunkPacket> packet)
{
    CommandRequest command;
    command.set_allocated_linklayer_tx(createLinkLayerTx(req, std::move(packet)).release());
    std::string serializedTransmission;
    command.SerializeToString(&serializedTransmission);
    mSocket.send(boost::asio::buffer(serializedTransmission));
    ROS_DEBUG("LinkLayerTransmission sent to CCU");
}
