#include "multicast_receiver.hpp"
#include "vanetza_context.hpp"
#include "cmc-ccu.pb.h"
#include <ros/console.h>
#include <string>

MulticastReceiver::MulticastReceiver(boost::asio::io_service& io, const boost::asio::ip::address& multicastAddress, const int port, VanetzaContext* context) :
    mSocket(io), mContext(context)
{
    boost::asio::ip::udp::endpoint listenEndpoint(boost::asio::ip::udp::v4(), port);
    mSocket.open(listenEndpoint.protocol());
    mSocket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    mSocket.bind(listenEndpoint);
    mSocket.set_option(boost::asio::ip::multicast::join_group(multicastAddress));
}

void MulticastReceiver::startReceive()
{
    mSocket.async_receive_from(
        boost::asio::buffer(mReceivedData), mEndpoint,
        boost::bind(&MulticastReceiver::handleReceive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
    );
}

void MulticastReceiver::handleReceive(const boost::system::error_code& ec, size_t bytes)
{
    if (!ec) {
        ROS_DEBUG_STREAM_NAMED("multicast", bytes << " bytes received from " << mEndpoint);

        vanetza::ByteBuffer buf(mReceivedData.begin(), mReceivedData.begin() + bytes);

        GossipMessage gossipMessage;
        gossipMessage.ParseFromArray(buf.data(), buf.size());

        switch (gossipMessage.kind_case()) {
            case GossipMessage::KindCase::kCbr:
                ROS_DEBUG_NAMED("multicast", "CBR report received from CCU");
                mContext->passCbrToRouter(gossipMessage.cbr());
                break;
            case GossipMessage::KindCase::kLinklayerRx:
                ROS_DEBUG_NAMED("multicast", "LinkLayerReception received from CCU");
                mContext->passMessageToRouter(std::unique_ptr<LinkLayerReception> {
                        gossipMessage.release_linklayer_rx() });
                break;
            default:
                ROS_ERROR_NAMED("multicast", "Received a CCU GossipMessage of unknown kind %i", gossipMessage.kind_case());
        }

        mSocket.async_receive_from(
            boost::asio::buffer(mReceivedData), mEndpoint,
            boost::bind(&MulticastReceiver::handleReceive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
    }
}
