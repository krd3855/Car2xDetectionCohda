#ifndef CP_TX_NODELET_HPP_3ON6JCAY
#define CP_TX_NODELET_HPP_3ON6JCAY

#include <boost/optional/optional.hpp>
#include <nodelet/nodelet.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <etsi_its_msgs/CPM.h>
#include <ros/timer.h>
#include <vanetza/asn1/cpm.hpp>
#include <cstdint>
#include "vanetza/LocalisationBody.h"
#include "vanetza/LocalisationStatus.h"
#include "vanetza/LocalisationVehicleInformation.h"
#include <sensor_msgs/NavSatFix.h>


namespace vanetza
{

class CpTxNodelet : public nodelet::Nodelet
{
public:
    CpTxNodelet();

    void onInit() override;

private:
    void checkTriggers(etsi_its_msgs::CPMConstPtr);
    void transmitMessage(etsi_its_msgs::CPMConstPtr);
    etsi_its_msgs::CPMPtr generateMessage(etsi_its_msgs::CPMConstPtr provided) const;


    boost::optional<int32_t> station_id_;
    ros::Duration cpm_rate_min_;
    ros::Duration cpm_rate_max_;
    ros::Duration cpm_rate_;
    unsigned cpm_rate_counter_max_;
    unsigned cpm_rate_counter_;
    ros::Time last_cpm_transmission_;
    ros::Subscriber sub_cpm_provider_;
    ros::Subscriber sub_id_change_;
    ros::Publisher pub_cpm_transmitter_;
    etsi_its_msgs::CPM::ConstPtr cpm_transmission_;
    ros::ServiceClient client_data_request_;
};

} // namespace vanetza

#endif /* CP_TX_NODELET_HPP_3ON6JCAY */