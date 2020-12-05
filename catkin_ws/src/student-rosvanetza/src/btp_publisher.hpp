#ifndef BTP_PUBLISHER_HPP_HZGX9BSY
#define BTP_PUBLISHER_HPP_HZGX9BSY

#include <ros/node_handle.h>
#include <vanetza/btp/data_indication.hpp>
#include <vanetza/geonet/transport_interface.hpp>

class BtpPublisher : public vanetza::geonet::TransportInterface
{
public:
    BtpPublisher(ros::NodeHandle*);
    void indicate(const vanetza::geonet::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;

private:
    void publish(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>);

    ros::Publisher publisher_;
};

#endif /* BTP_PUBLISHER_HPP_HZGX9BSY */
