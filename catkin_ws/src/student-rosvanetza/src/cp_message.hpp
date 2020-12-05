#ifndef CP_MESSAGE_HPP_1TS8FOUY
#define CP_MESSAGE_HPP_1TS8FOUY

#include <boost/shared_ptr.hpp>
#include <etsi_its_msgs/CPM.h>
#include <string>

namespace vanetza
{

namespace asn1 {class Cpm; }

/**
 * Convert ASN.1 data structure to ROS etsi_its_msg CPM
 * \param cpm ASN.1 data structure
 * \param msg optional error string
 * \return converted CPM (or nullptr on error)
 */
boost::shared_ptr<etsi_its_msgs::CPM> convert_cpm(const asn1::Cpm& cpm, std::string* msg = nullptr);

/**
 * Convert ROS etsi_its_msg CPM to ASN.1 data structure
 * \param ptr etsi_its_msgs CPM
 * \return converted CPM
 */
asn1::Cpm convertCpm(etsi_its_msgs::CPMConstPtr ptr);
} // namespace  vanetza

#endif /* CP_MESSAGE_HPP_1TS8FOUY */

