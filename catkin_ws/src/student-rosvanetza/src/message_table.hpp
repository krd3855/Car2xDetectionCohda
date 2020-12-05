#ifndef MESSAGE_TABLE_HPP_4DRUFN1X
#define MESSAGE_TABLE_HPP_4DRUFN1X

#include <etsi_its_msgs/ActionID.h>
#include <ros/message_forward.h>
#include <map>

namespace etsi_its_msgs
{
ROS_DECLARE_MESSAGE(ManagementContainer);
} // namespace etsi_its_msgs

namespace vanetza
{

class MessageTable
{
public:
    void trigger(const etsi_its_msgs::ManagementContainer&);
    bool update(const etsi_its_msgs::ManagementContainer&);
    void terminate(const etsi_its_msgs::ManagementContainer&);
    bool exists(const etsi_its_msgs::ActionID&) const;
    void purge();

private:
    struct CompareActionId
    {
        bool operator()(const etsi_its_msgs::ActionID&, const etsi_its_msgs::ActionID&) const;
    };
    std::map<etsi_its_msgs::ActionID, ros::Time, CompareActionId> entries_;
};

} // namespace vanetza

#endif /* MESSAGE_TABLE_HPP_4DRUFN1X */

