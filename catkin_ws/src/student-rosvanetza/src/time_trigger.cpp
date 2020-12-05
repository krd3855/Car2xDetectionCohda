#include "time_trigger.hpp"
#include <ros/console.h>
#include <functional>

namespace asio = boost::asio;
using namespace vanetza;

TimeTrigger::TimeTrigger(asio::io_service& io_service) :
    strand_(io_service), timer_(io_service),
    runtime_(time_point_cast<vanetza::Clock>(tai_clock::now())),
    scheduled_(vanetza::Clock::time_point::max())
{
    strand_.dispatch([this]() { schedule(); });
}

void TimeTrigger::schedule(Clock::time_point tp, const Callback& cb, const void* scope)
{
    strand_.dispatch([=]() {
        runtime_.schedule(tp, cb, scope);
        schedule();
    });
}

void TimeTrigger::schedule(Clock::duration d, const Callback& cb, const void* scope)
{
    strand_.post([=]() {
        runtime_.trigger(now());
        runtime_.schedule(d, cb, scope);
        schedule();
    });
}

void TimeTrigger::cancel(const void* scope)
{
    strand_.dispatch([=]() {
        runtime_.cancel(scope);
        schedule();
    });
}

Clock::time_point TimeTrigger::now() const
{
    return time_point_cast<vanetza::Clock>(tai_clock::now());
}

void TimeTrigger::schedule()
{
    auto next = runtime_.next();
    if (next < scheduled_) {
        scheduled_ = next;
        timer_.expires_at(time_point_cast<tai_clock>(next));
        timer_.async_wait(strand_.wrap(std::bind(&TimeTrigger::on_timeout, this, std::placeholders::_1)));
    }
}

void TimeTrigger::on_timeout(const boost::system::error_code& ec)
{
    runtime_.trigger(now());

    if (ec) {
        if (ec != boost::asio::error::operation_aborted) {
            ROS_ERROR("Problem with TimeTrigger's internal timer: %s", ec.message().c_str());
        }
    } else {
        scheduled_ = vanetza::Clock::time_point::max();
        schedule();
    }
}
