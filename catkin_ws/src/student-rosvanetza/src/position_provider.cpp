#include "position_provider.hpp"
#include <boost/bind.hpp>
#include <cmath>

PositionProvider::PositionProvider(const vanetza::Runtime& rt, ros::NodeHandle& nh) :
    m_runtime(rt)
{
    m_subscriber = nh.subscribe<vanetza::PositionVector>("position_vector", 1,
            boost::bind(&PositionProvider::callback, this, _1));
}

const vanetza::PositionFix& PositionProvider::position_fix()
{
    std::unique_lock<std::mutex> lock(m_mutex, std::defer_lock);
    if (lock.try_lock()) {
        m_fix_returned = m_fix_staging;
    }

    return m_fix_returned;
}

void PositionProvider::callback(vanetza::PositionVectorConstPtr posfix)
{
    if (!std::isfinite(posfix->latitude) || !std::isfinite(posfix->longitude)) {
        ROS_WARN_THROTTLE(1, "ignore position vector with infinite coordinates");
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    using namespace vanetza::units;
    static const TrueNorth north = TrueNorth::from_value(0.0);

    m_fix_staging.timestamp = m_runtime.now(); // TODO convert header's stamp (ROS -> TAI)
    m_fix_staging.latitude = posfix->latitude * degree;
    m_fix_staging.longitude = posfix->longitude * degree;
    m_fix_staging.confidence.semi_major = posfix->semi_major_confidence * si::meter;
    m_fix_staging.confidence.semi_minor = posfix->semi_minor_confidence * si::meter;
    m_fix_staging.confidence.orientation = north + posfix->semi_major_orientation * degree;
    m_fix_staging.speed.assign(posfix->speed * si::meter_per_second, posfix->speed_confidence * si::meter_per_second);
    m_fix_staging.course.assign(north + posfix->course * degree, north + posfix->course_confidence * degree);
}
