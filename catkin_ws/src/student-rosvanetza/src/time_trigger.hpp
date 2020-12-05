#ifndef TIME_TRIGGER_HPP_XRPGDYXO
#define TIME_TRIGGER_HPP_XRPGDYXO

#include "tai_clock.hpp"
#include <vanetza/common/manual_runtime.hpp>
#include <boost/asio/basic_waitable_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/strand.hpp>
#include <mutex>

class TimeTrigger : public vanetza::Runtime
{
public:
    TimeTrigger(boost::asio::io_service&);

    // vanetza::Runtime interface
    void schedule(vanetza::Clock::time_point, const Callback&, const void* scope = nullptr) override;
    void schedule(vanetza::Clock::duration, const Callback&, const void* scope = nullptr) override;
    void cancel(const void* scope) override;
    vanetza::Clock::time_point now() const override;

private:
    void schedule();
    void on_timeout(const boost::system::error_code&);

    boost::asio::io_service::strand strand_;
    boost::asio::basic_waitable_timer<tai_clock> timer_;
    vanetza::ManualRuntime runtime_;
    vanetza::Clock::time_point scheduled_;
};

#endif /* TIME_TRIGGER_HPP_XRPGDYXO */

