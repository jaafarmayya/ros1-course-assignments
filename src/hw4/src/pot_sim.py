#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

class PotSimulator:
    def __init__(self):
        rospy.init_node('pot_simulator', anonymous=False)
        self._pub = rospy.Publisher('/pot', Float32, queue_size=10)
        self._rate = rospy.Rate(50)
        self._cycle_time = rospy.get_param('~cycle_time', 20.0)

    def run(self):
        while not rospy.is_shutdown():
            t = rospy.get_time()
            value = (math.sin(2 * math.pi * t / self._cycle_time) + 1.0) * 0.5 * 1023.0
            msg = Float32(data=value)
            self._pub.publish(msg)
            self._rate.sleep()

if __name__ == '__main__':
    try:
        PotSimulator().run()
    except rospy.ROSInterruptException:
        pass
