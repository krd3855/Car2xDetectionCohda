#ifndef BTP_DATA_SERVICE_HPP_MLZQDXAP
#define BTP_DATA_SERVICE_HPP_MLZQDXAP

#include "vanetza_context.hpp"
#include "vanetza/BtpData.h"
#include <ros/node_handle.h>
#include <ros/service.h>

class BtpDataService
{
public:
    BtpDataService(ros::NodeHandle*, VanetzaContext& stack);
    bool callback(vanetza::BtpData::Request&, vanetza::BtpData::Response&);

private:
    VanetzaContext& stack_;
    ros::ServiceServer service_;
};

#endif /* BTP_DATA_SERVICE_HPP_MLZQDXAP */

