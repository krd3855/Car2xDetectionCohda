#include "security_id_publisher.hpp"
#include "vanetza/SecurityIdChange.h"
#include <ros/exception.h>
#include <algorithm>
#include <cassert>

SecurityIdPublisher::SecurityIdPublisher(ros::NodeHandle* nh) :
    publisher_(nh->advertise<vanetza::SecurityIdChange>("id_change", 1, true))
{
    if (!publisher_)
    {
        throw ros::Exception("Instantiation of id_change publisher failed");
    }
}

void SecurityIdPublisher::publishCommit(const vanetza::security::HashedId8& id)
{
    boost::shared_ptr<vanetza::SecurityIdChange> change = boost::make_shared<vanetza::SecurityIdChange>();
    change->phase = vanetza::SecurityIdChange::PHASE_COMMIT;
    static_assert(sizeof(decltype(vanetza::SecurityIdChange::id)::elems) == id.size(),
        "length of pseudonym types does not match");
    std::copy(id.begin(), id.end(), change->id.begin());
    publisher_.publish(change);
}
