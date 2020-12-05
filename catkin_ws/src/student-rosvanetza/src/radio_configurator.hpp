#ifndef RADIO_CONFIGURATOR_HPP_9YZSWUCK
#define RADIO_CONFIGURATOR_HPP_9YZSWUCK

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/strand.hpp>
#include <boost/optional.hpp>
#include <boost/system/error_code.hpp>
#include <vanetza/net/mac_address.hpp>
#include <string>

class RadioConfiguration;

class RadioConfigurator
{
public:
    RadioConfigurator(boost::asio::io_service&);
    void setLinkLayerAddress(const vanetza::MacAddress&);
    void setDefaultTxPower(double dbm);
    void run(const boost::asio::ip::udp::endpoint&);

private:
    void configure(boost::asio::yield_context, bool*);
    void timeout(boost::asio::yield_context, bool*);

    boost::asio::io_service& mIoService;
    boost::asio::io_service::strand mStrand;
    boost::asio::ip::udp::socket mSocket;
    boost::asio::deadline_timer mTimer;
    boost::optional<vanetza::MacAddress> mLinkLayerAddr;
    double mTxPower; /*< default transmission power in dBm */
};

#endif /* RADIO_CONFIGURATOR_HPP_9YZSWUCK */

