#ifndef CA_MESSAGE_HPP_1TS8FOUY
#define CA_MESSAGE_HPP_1TS8FOUY

#include <boost/shared_ptr.hpp>
#include <etsi_its_msgs/CAM.h>
#include <string>

namespace vanetza
{

namespace asn1 { class Cam; }

/**
 * Convert ASN.1 data structure to ROS etsi_its_msg CAM
 * \param cam ASN.1 data structure
 * \param msg optional error string
 * \return converted CAM (or nullptr on error)
 */
boost::shared_ptr<etsi_its_msgs::CAM> convertCam(const asn1::Cam& cam, std::string* msg = nullptr);

/**
 * Convert ROS etsi_its_msg CAM to ASN.1 data structure
 * \param ptr etsi_its_msgs CAM
 * \return converted CAM
 */
asn1::Cam convertCam(etsi_its_msgs::CAMConstPtr ptr);

} // namespace  vanetza

#endif /* CA_MESSAGE_HPP_1TS8FOUY */

