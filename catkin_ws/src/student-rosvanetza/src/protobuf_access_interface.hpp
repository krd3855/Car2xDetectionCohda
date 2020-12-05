#ifndef PROTOBUF_REQUEST_INTERFACE_HPP_
#define PROTOBUF_REQUEST_INTERFACE_HPP_

#include "vanetza/access/interface.hpp"
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/io_service.hpp>

class ProtobufAccessInterface : public vanetza::access::Interface
{
public:
    ProtobufAccessInterface(boost::asio::io_service&, const boost::asio::ip::udp::endpoint& ccu);
    void request(const vanetza::access::DataRequest&, std::unique_ptr<vanetza::ChunkPacket>) override;

private:
    boost::asio::ip::udp::socket mSocket;
};

#endif /* PROTOBUF_REQUEST_INTERFACE_HPP_ */
