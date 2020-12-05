#include "radio_configurator.hpp"
#include "cmc-ccu.pb.h"
#include <ros/console.h>
#include <functional>

void printConfiguration(const RadioConfiguration&);

class JobGuard
{
public:
    JobGuard(bool* done) : done_(done) {}
    ~JobGuard() { if (done_) *done_ = true; }

private:
    bool* done_;
};

class TimeoutGuard
{
public:
    TimeoutGuard(boost::asio::deadline_timer& timer) : timer_(timer) {}
    ~TimeoutGuard() { timer_.cancel(); }

private:
    boost::asio::deadline_timer& timer_;
};

RadioConfigurator::RadioConfigurator(boost::asio::io_service& io) :
    mIoService(io), mStrand(mIoService), mSocket(mIoService), mTimer(mIoService),
    mTxPower(std::numeric_limits<double>::quiet_NaN())
{
}

void RadioConfigurator::setLinkLayerAddress(const vanetza::MacAddress& addr)
{
    mLinkLayerAddr = addr;
}

void RadioConfigurator::setDefaultTxPower(double power_dbm)
{
    mTxPower = power_dbm;
}

void RadioConfigurator::run(const boost::asio::ip::udp::endpoint& ccu)
{
    mSocket.connect(ccu);
    mTimer.expires_from_now(boost::posix_time::seconds(3));

    bool config_done = false;
    bool timeout_reached = false;
    boost::asio::spawn(mStrand, std::bind(&RadioConfigurator::configure, this, std::placeholders::_1, &config_done));
    boost::asio::spawn(mStrand, std::bind(&RadioConfigurator::timeout, this, std::placeholders::_1, &timeout_reached));

    while (!config_done || !timeout_reached)
    {
        mIoService.run_one();
    }
}

void RadioConfigurator::configure(boost::asio::yield_context yield, bool* finished)
{
    JobGuard job_guard(finished);
    TimeoutGuard timeout_guard(mTimer);

    CommandRequest cmd_request;
    RadioConfiguration* radio_cfg = cmd_request.mutable_radio_cfg();
    if (mLinkLayerAddr)
    {
        const vanetza::MacAddress& addr = mLinkLayerAddr.get();
        radio_cfg->set_address(addr.octets.data(), addr.octets.size());
    }
    if (std::isfinite(mTxPower))
    {
        radio_cfg->set_default_tx_power_cbm(std::round(mTxPower * 10.0));
    }
    radio_cfg->set_filter_unicast_destination(true);

    std::string tx_buffer;
    cmd_request.SerializeToString(&tx_buffer);
    boost::system::error_code ec;
    mSocket.async_send(boost::asio::buffer(tx_buffer), yield[ec]);
    if (ec)
    {
        ROS_ERROR_STREAM("Sending radio configuration to CCU failed: " << ec.message());
        return;
    }

    std::string rx_buffer(2048, '\0');
    std::size_t received = mSocket.async_receive(boost::asio::buffer(&rx_buffer.front(), rx_buffer.size()), yield[ec]);
    if (ec)
    {
        ROS_ERROR_STREAM("No CommandResponse received from CCU: " << ec.message());
        return;
    }

    rx_buffer.resize(received);
    CommandResponse cmd_response;
    if (cmd_response.ParseFromString(rx_buffer))
    {
        if (cmd_response.status() != CommandResponse::SUCCESS)
        {
            const std::string code = CommandResponse_Status_Name(cmd_response.status());
            const std::string msg = cmd_response.has_message() ? cmd_response.message() : "[no details given]";
            ROS_ERROR("Setting MAC address of CCU failed (%s): %s", code.c_str(), msg.c_str());
        }
        else if (cmd_response.has_data() && cmd_response.data().has_radio_cfg())
        {
            printConfiguration(cmd_response.data().radio_cfg());
        }
    }
    else
    {
        ROS_ERROR("Could not parse CommandResponse from CCU");
    }

    mTimer.cancel();
    mSocket.close();
}

void RadioConfigurator::timeout(boost::asio::yield_context yield, bool* finished)
{
    JobGuard guard(finished);

    boost::system::error_code ec;
    mTimer.async_wait(yield[ec]);
    if (!ec && mSocket.is_open())
    {
        ROS_ERROR("Radio configuration aborted due to timeout");
        mSocket.close();
    }
}

void printConfiguration(const RadioConfiguration& radio_cfg)
{
    if (radio_cfg.has_channel_frequency_mhz())
    {
        ROS_INFO("CCU is tuned to the %.3f GHz channel", radio_cfg.channel_frequency_mhz() / 1000.0);
    }

    if (radio_cfg.has_default_tx_power_cbm())
    {
        ROS_INFO("CCU's default transmission power is %.1f dBm", radio_cfg.default_tx_power_cbm() / 10.0);
    }

    if (radio_cfg.has_default_tx_datarate_500kbps())
    {
        ROS_INFO("CCU's default transmission data rate is %.0f Mbit/s", radio_cfg.default_tx_datarate_500kbps() * 0.5);
    }
}
