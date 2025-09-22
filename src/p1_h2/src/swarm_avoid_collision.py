#!/usr/bin/env python
import rospy
import threading
import random
import math
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rospy.exceptions import ROSException

class TurtleBot():
    def __init__(self, name, initial_x, initial_y, initial_theta, all_agents):
        self.name = name
        self.position = Pose()
        self.peers = all_agents
        self.min_dist = 0.7
        self.speed = 1.5
        self.ang_noise = 1.0

        self.wall_margin = 1.0
        self.turning_wall = False
        self.target_theta = 0.0

        try:
            rospy.wait_for_service('kill', timeout=1.0)
            rospy.ServiceProxy('kill', Kill)(name)
        except (ROSException, rospy.ServiceException):
            pass

        rospy.wait_for_service('spawn')
        rospy.ServiceProxy('spawn', Spawn)(initial_x, initial_y, initial_theta, name)

        self.pose_sub = rospy.Subscriber(f"{name}/pose", Pose, self.update_pose)
        self.vel_pub  = rospy.Publisher(f"{name}/cmd_vel", Twist, queue_size=1)
        self.rate     = rospy.Rate(10)

    def update_pose(self, msg):
        self.position = msg

    def avoid_collisions(self):
        closest, min_d2 = None, float('inf')
        for p in self.peers:
            if p is self:
                continue
            dx = p.position.x - self.position.x
            dy = p.position.y - self.position.y
            d2 = dx*dx + dy*dy
            if d2 < min_d2:
                min_d2, closest = d2, p

        if closest and min_d2 < (self.min_dist**2):
            away_theta = math.atan2(
                self.position.y - closest.position.y,
                self.position.x - closest.position.x
            )
            err = self.normalize_angle(away_theta - self.position.theta)
            return 3.0 * err
        return None

    def near_wall(self):
        x, y = self.position.x, self.position.y
        return (
            x < self.wall_margin or
            x > 11.0 - self.wall_margin or
            y < self.wall_margin or
            y > 11.0 - self.wall_margin
        )

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def wander(self):
        twist = Twist()
        while not rospy.is_shutdown():
            avoid_omega = self.avoid_collisions()
            if avoid_omega is not None:
                twist.linear.x  = 0.5 * self.speed
                twist.angular.z = avoid_omega
            elif self.turning_wall:
                delta = self.normalize_angle(self.target_theta - self.position.theta)
                if abs(delta) > 0.05:
                    twist.linear.x  = 0.0
                    twist.angular.z = 2.0 * delta
                else:
                    self.turning_wall = False
                    twist.linear.x  = self.speed
                    twist.angular.z = 0.0
            elif self.near_wall():
                self.turning_wall = True
                self.target_theta = self.normalize_angle(self.position.theta + math.pi)
                twist.linear.x  = 0.0
                spin_dir = math.copysign(
                    1.0,
                    self.normalize_angle(self.target_theta - self.position.theta)
                )
                twist.angular.z = spin_dir * self.ang_noise
            else:
                twist.linear.x  = self.speed
                twist.angular.z = random.uniform(-self.ang_noise, self.ang_noise)

            self.vel_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Swarm_Turtles')
    names   = rospy.get_param('~names',      ['turtle1','turtle2','turtle3'])
    inits_x = rospy.get_param('~initial_x',  [2.0,5.0,8.0])
    inits_y = rospy.get_param('~initial_y',  [2.0,5.0,8.0])
    inits_th= rospy.get_param('~initial_th', [0.0,0.0,0.0])

    agents = []
    for n, x, y, th in zip(names, inits_x, inits_y, inits_th):
        agents.append(TurtleBot(n, x, y, th, agents))

    for bot in agents:
        t = threading.Thread(target=bot.wander)
        t.daemon = True
        t.start()

    rospy.spin()
