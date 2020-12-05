#include "message_table.hpp"
#include <etsi_its_msgs/ManagementContainer.h>
#include <algorithm>

namespace vanetza
{

using namespace etsi_its_msgs;

ros::Duration getDuration(const ManagementContainer& mgmt)
{
    double validity_sec = mgmt.validity_duration / ManagementContainer::VALIDITY_DURATION_ONE_SECOND_AFTER_DETECTION;
    if (mgmt.detection_time < mgmt.reference_time)
    {
        validity_sec -= (mgmt.reference_time - mgmt.detection_time) / 1000.0;
    }
    return ros::Duration { std::max(0.0, validity_sec) };
}

void MessageTable::trigger(const ManagementContainer& mgmt)
{
    entries_.emplace(mgmt.action_id, ros::Time::now() + getDuration(mgmt));
}

bool MessageTable::update(const ManagementContainer& mgmt)
{
    auto found = entries_.find(mgmt.action_id);
    if (found != entries_.end())
    {
        found->second = ros::Time::now() + getDuration(mgmt);
        return true;
    }
    else
    {
        return false;
    }
}

void MessageTable::terminate(const ManagementContainer& mgmt)
{
    // same as update for now because we do not track state (active, cancelled, negated) yet
    update(mgmt);
}

bool MessageTable::exists(const ActionID& action_id) const
{
    auto found = entries_.find(action_id);
    return found != entries_.end();
}

void MessageTable::purge()
{
    auto now = ros::Time::now();
    for (auto it = entries_.begin(); it != entries_.end();)
    {
        if (it->second < now)
        {
            it = entries_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

bool MessageTable::CompareActionId::operator()(const ActionID& a, const ActionID& b) const
{
    if (a.station_id == b.station_id)
    {
        return a.sequence_number < b.sequence_number;
    }
    else
    {
        return a.station_id < b.station_id;
    }
}

} // namespace vanetza
