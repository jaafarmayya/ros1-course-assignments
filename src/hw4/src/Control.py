#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray

class AltitudeController:
    def __init__(self):
        rospy.init_node('altitude_control', anonymous=False)
        self._latest_pot = None
        self._alt_pub = rospy.Publisher('/altitude', Float32MultiArray, queue_size=10)
        self._marker_pub = rospy.Publisher('/create_sphere', Marker, queue_size=10)
        rospy.Subscriber('/pot', Float32, self._pot_callback)
        self._rate = rospy.Rate(1)
        self._min_z = rospy.get_param('zmin', 0.0)
        self._max_z = rospy.get_param('zmax', 1.0)
        self._tol = rospy.get_param('tolerance', 0.0)

    def _pot_callback(self, msg):
        self._latest_pot = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if self._latest_pot is None:
                self._rate.sleep()
                continue

            z_val = self._min_z + (self._latest_pot / 1023.0) * (self._max_z - self._min_z)

            sphere = Marker()
            sphere.header.frame_id = 'world'
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'altitude_marker'
            sphere.id = 0
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.z = z_val
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.0
            sphere.color.r = 1.0
            sphere.color.a = 1.0

            data_msg = Float32MultiArray()
            data_msg.data = [z_val, self._tol]

            self._alt_pub.publish(data_msg)
            self._marker_pub.publish(sphere)
            self._rate.sleep()

if __name__ == '__main__':
    try:
        AltitudeController().run()
    except rospy.ROSInterruptException:
        pass
