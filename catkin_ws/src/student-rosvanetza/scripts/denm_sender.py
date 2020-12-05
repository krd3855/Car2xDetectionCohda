#!/usr/bin/env python

import rospy
import vanetza.srv as vsrv
import vanetza.msg as vanetza
import etsi_its_msgs.msg as its_msg


class DenmSender:

    def __init__(self):
        self.rate = rospy.Rate(1)
        self.tx = rospy.ServiceProxy("/den_request", vsrv.DenmTx)
        self.latitude = 48.766083
        self.longitude = 11.434416

    def loop(self):
        while not rospy.is_shutdown():
            self.send_denm()
            self.rate.sleep()

    def send_denm(self):
        req = vsrv.DenmTxRequest()
        req.request = vsrv.DenmTxRequest.REQUEST_TRIGGER
        req.denm = self.generate_message()
        req.traffic_class = self.generate_traffic_class()
        req.repetition_interval = rospy.Duration(0)
        req.repetition_duration = rospy.Duration(0)
        req.destination_area = self.generate_circle_area(500.0)
        try:
            resp = self.tx(req)
            rospy.loginfo("{}: response: {}".format(rospy.get_name(), resp))
        except rospy.ServiceException as e:
            rospy.logerr("{}: service error: {}".format(rospy.get_name(), str(e)))

    def generate_circle_area(self, radius):
        area = vanetza.GeoNetArea()
        area.type = vanetza.GeoNetArea.TYPE_CIRCLE
        area.latitude = self.latitude
        area.longitude = self.longitude
        area.distance_a = radius
        return area

    def generate_traffic_class(self):
        tc = vanetza.TrafficClass()
        tc.id = vanetza.TrafficClass.ID_DCC_DP1
        return tc

    def generate_message(self):
        time_now = rospy.get_rostime()
        start_its_epoch = 1072915200  # POSIX seconds at start of ITS epoch (2004-01-01)
        start_its_time = rospy.Time.from_seconds(start_its_epoch)
        elapsed_ms = (time_now - start_its_time).to_sec() * 1000.0
        # NOTE: above time calculation ignores leap seconds but it is okay for this script

        denm = its_msg.DENM()
        denm.header.stamp = time_now
        denm.its_header.station_id = 4711
        denm.its_header.message_id = its_msg.ItsPduHeader.MESSAGE_ID_DENM
        denm.its_header.protocol_version = 2

        management = its_msg.ManagementContainer()
        management.detection_time = int(elapsed_ms)
        management.reference_time = int(elapsed_ms)
        management.event_position.latitude = int(self.latitude * 1e7)
        management.event_position.longitude = int(self.longitude * 1e7)
        print(management.event_position.latitude )
        print(management.event_position.longitude )
        management.validity_duration = 2
        management.relevance_distance.value = its_msg.RelevanceDistance.LESS_THAN_50M
        management.relevance_traffic_direction.value = its_msg.RelevanceTrafficDirection.ALL_TRAFFIC_DIRECTIONS
        management.termination = its_msg.ManagementContainer.TERMINATION_NEGATION
        denm.management = management

        situation = its_msg.SituationContainer()
        situation.information_quality.value = 3
        situation.event_type.cause_code = its_msg.CauseCode.DANGEROUS_SITUATION
        situation.event_type.sub_cause_code = its_msg.CauseCode.TRAFFIC_CONDITION
        denm.situation = situation
        denm.has_situation = True

        location = its_msg.LocationContainer()
        location.event_speed.confidence = 10
        location.event_position_heading.confidence = 10
        denm.location = location
        denm.has_location = True

        return denm


if __name__ == '__main__':
    try:
        rospy.init_node('denm_sender')
        sender = DenmSender()
        sender.loop()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
