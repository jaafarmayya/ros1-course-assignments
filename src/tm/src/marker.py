#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from visualization_msgs.msg import Marker

class MarkerCube:
    def __init__(self):
        self.state_pub = rospy.Publisher("/visualization_marker",
                                         Marker,
                                         queue_size=1,
                                         latch=True)
        rospy.Subscriber("/turtle1/pose",
                         Pose,
                         self.update_pose)
        self.cx = 5.5
        self.cy = 5.5

        self.marker = Marker()
        self.marker.header.frame_id = "map"      
        self.marker.ns = "turtle_cube"
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.publish_marker(0.0, 0.0)

       

    def publish_marker(self, x, y):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0.0
        self.state_pub.publish(self.marker)

    def update_pose(self, pose):
        x_map = pose.x - self.cx
        y_map = pose.y - self.cy
        self.publish_marker(x_map, y_map)

if __name__ == "__main__":
    rospy.init_node("marker_cube", anonymous=True)
    MarkerCube()
    rospy.spin()
