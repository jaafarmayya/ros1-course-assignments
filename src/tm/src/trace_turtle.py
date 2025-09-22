import rospy
from turtlesim.srv import *
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from rospy.exceptions import ROSException

rospy.init_node('Swarm_Turtles')

class TurtleBot():
    def __init__(self, name, initial_x, initial_y, initial_theta):
        self.name = name

        try:
            rospy.wait_for_service('kill', timeout=1.0)
            rospy.ServiceProxy('kill', Kill)(name)  
        except (ROSException, rospy.ServiceException):
            pass

        self.spawnTurtle(name, initial_x, initial_y, initial_theta)
        self.position = Pose()
        self.pose_subscriber = rospy.Subscriber(f'{name}/pose', Pose, self.updatePose)
        self.marker_pose_publisher = rospy.Publisher(f"{name}/marker_pose", Pose, queue_size=10)
        self.rate = rospy.Rate(10)

    def spawnTurtle(self, name, x,y,theta):
        rospy.wait_for_service('spawn')
        server = rospy.ServiceProxy('spawn', Spawn)
        server(x,y,theta,name)

    def updatePose(self, data):
        self.position.x = data.x
        self.position.y = data.y
        self.position.theta = data.theta
        self.publish(data)
    
    def publish(self, data):
        msg = Pose()
        msg.x = data.x
        msg.y = data.y
        msg.z = data.z
        self.marker_pose_publisher.publish(msg)


if __name__ == '__main__':
    name = "turtle1"
    initial_x = 5.5
    initial_y = 5.5
    turtle = TurtleBot(name, initial_x, initial_y, 0)
    rospy.spin()
