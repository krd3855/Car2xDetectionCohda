#!/usr/bin/env python

import roslib
import rospy
from cmc_localisation.msg import LocalisationEllipsoid, LocalisationVehicleInformation

roslib.load_manifest('vanetza')


class GpsMock:

    def __init__(self):
        self.rate = rospy.Rate(1)
        self.pub_ellipsoid = rospy.Publisher('localisation_ellipsoid', LocalisationEllipsoid, queue_size=5)
        self.pub_vehicle = rospy.Publisher('localisation_vehicle', LocalisationVehicleInformation, queue_size=5)
        self.latitude = rospy.get_param("~latitude")
        self.longitude = rospy.get_param("~longitude")
        self.altitude = rospy.get_param("~altitude", 0.0)

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            ellipsoid = LocalisationEllipsoid()
            ellipsoid.header.stamp = now
            ellipsoid.latitude = self.latitude
            ellipsoid.longitude = self.longitude
            ellipsoid.semi_major_confidence = 10.0
            ellipsoid.semi_minor_confidence = 10.0
            ellipsoid.semi_major_orientation = 0.0
            ellipsoid.altitude = self.altitude
            ellipsoid.altitude = 10.0
            self.pub_ellipsoid.publish(ellipsoid)

            vehicle = LocalisationVehicleInformation()
            vehicle.header.stamp = now
            vehicle.speed = 0.0
            vehicle.speed_confidence = 0.0
            vehicle.heading = 0.0
            vehicle.heading_confidence = 0.0
            vehicle.curvature = 0.0
            vehicle.curvature_confidence = 0.0
            self.pub_vehicle.publish(vehicle)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('gps_mock')
        GpsMock().run()

    except rospy.ROSInterruptException:
        pass
