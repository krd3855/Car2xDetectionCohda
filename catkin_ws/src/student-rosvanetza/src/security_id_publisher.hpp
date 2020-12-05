#ifndef SECURITY_ID_PUBLISHER_HPP_PGMYUEIB
#define SECURITY_ID_PUBLISHER_HPP_PGMYUEIB

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <vanetza/security/basic_elements.hpp>

class SecurityIdPublisher
{
public:
    SecurityIdPublisher(ros::NodeHandle*);
    void publishCommit(const vanetza::security::HashedId8&);

private:
    ros::Publisher publisher_;
};

#endif /* SECURITY_ID_PUBLISHER_HPP_PGMYUEIB */
