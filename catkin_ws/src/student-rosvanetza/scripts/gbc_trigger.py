#!/usr/bin/env python

import roslib
import rospy
from vanetza.srv import BtpData, BtpDataRequest, BtpDataResponse
from vanetza.msg import GeoNetArea, GeoNetDestination, TrafficClass

roslib.load_manifest('vanetza')


class GbcTrigger:

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~rate", 10))
        self.repetitions = rospy.get_param("~repetitions", 1)
        self.radius = rospy.get_param("~radius", 100.0)
        self.latitude = rospy.get_param("~latitude")
        self.longitude = rospy.get_param("~longitude")
        self.client = rospy.ServiceProxy('btp_request', BtpData)
        self.count = 0

    def run(self):
        while not rospy.is_shutdown():
            request = BtpDataRequest()
            request.btp_type = BtpDataRequest.BTP_TYPE_B
            request.destination_port = 1234
            request.transport_type = BtpDataRequest.TRANSPORT_TYPE_GBC
            request.destination.type = GeoNetDestination.TYPE_AREA
            request.destination.area.type = GeoNetArea.TYPE_CIRCLE
            request.destination.area.latitude = self.latitude
            request.destination.area.longitude = self.longitude
            request.destination.area.distance_a = self.radius
            request.lifetime = rospy.Duration.from_sec(30.0)
            request.traffic_class.store_carry_forwarding = False
            request.traffic_class.channel_offloading = False
            request.traffic_class.id = TrafficClass.ID_DCC_DP1

            response = self.client(request)
            if response.confirm != BtpDataResponse.CONFIRM_ACCEPTED:
                rospy.logerror("BTP data request has been rejected")

            self.count += 1
            if self.count < self.repetitions:
                self.rate.sleep()
            else:
                rospy.signal_shutdown('done')


if __name__ == "__main__":
    try:
        rospy.init_node('trigger_gbc')
        if rospy.has_param('~delay'):
            delay = rospy.get_param('~delay')
            rospy.loginfo('waiting for %f seconds now', delay)
            rospy.sleep(delay)

        rospy.loginfo('waiting for btp_request service')
        rospy.wait_for_service('btp_request')
        rospy.loginfo('btp_request is available now')
        GbcTrigger().run()

    except rospy.ROSInterruptException:
        pass
