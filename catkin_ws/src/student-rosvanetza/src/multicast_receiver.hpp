#ifndef MULTICASTRECEIVER_HPP_
#define MULTICASTRECEIVER_HPP_

#include <boost/asio.hpp>
#include <array>
#include <cstdint>

class VanetzaContext;

class MulticastReceiver
{
public:
    MulticastReceiver(boost::asio::io_service&, const boost::asio::ip::address&, const int, VanetzaContext*);
    void handleReceive(const boost::system::error_code& error, size_t bytes);
    void startReceive();

private:
    boost::asio::ip::udp::socket mSocket;
    boost::asio::ip::udp::endpoint mEndpoint;

    std::array<uint8_t, 4096> mReceivedData;
    VanetzaContext* mContext;
};

#endif /* MULTICASTRECEIVER_HPP_ */
