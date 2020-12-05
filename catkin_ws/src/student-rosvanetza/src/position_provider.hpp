#ifndef POSITION_PROVIDER_HPP_RUEURJXL
#define POSITION_PROVIDER_HPP_RUEURJXL

#include "vanetza/PositionVector.h"
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>
#include <mutex>

class PositionProvider : public vanetza::PositionProvider
{
public:
    PositionProvider(const vanetza::Runtime&, ros::NodeHandle&);

    const vanetza::PositionFix& position_fix() override;

private:
    void callback(vanetza::PositionVectorConstPtr);

    const vanetza::Runtime& m_runtime;
    vanetza::PositionFix m_fix_returned;
    vanetza::PositionFix m_fix_staging;
    std::mutex m_mutex;
    ros::Subscriber m_subscriber;
};

#endif /* POSITION_PROVIDER_HPP_RUEURJXL */
