#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray

class ObstacleVisualizer:
    def __init__(self):
        rospy.init_node('obstacle_visualizer', anonymous=False)
        self.obstacle_x = rospy.get_param('obstacles_x', [])
        self.obstacle_y = rospy.get_param('obstacles_y', [])
        self.start_pose = rospy.get_param('starting_pose', None)
        self.goal_pose = rospy.get_param('goal_pose', None)
        self._publisher = rospy.Publisher('obstacles', MarkerArray, queue_size=10)
        self._markers = MarkerArray()
        self._create_markers()

    def _create_markers(self):
        for idx, (x, y) in enumerate(zip(self.obstacle_x, self.obstacle_y)):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = rospy.Time.now()
            m.ns = "obstacles"
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            self._markers.markers.append(m)

    def run(self, publish_rate_hz=1):
        rate = rospy.Rate(publish_rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            for marker in self._markers.markers:
                marker.header.stamp = now
            self._publisher.publish(self._markers)
            rate.sleep()

if __name__ == "__main__":
    try:
        viz = ObstacleVisualizer()
        viz.run()
    except rospy.ROSInterruptException:
        pass
