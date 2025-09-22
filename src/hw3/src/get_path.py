#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathGenerator:
    def __init__(self):
        rospy.init_node('path_generator', anonymous=False)
        self.path_pub = rospy.Publisher('robot_path', Path, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher('robot_pose', Marker, queue_size=10)
        self.rate = rospy.Rate(20)
        xs = rospy.get_param('obstacles_x', [])
        ys = rospy.get_param('obstacles_y', [])
        self.start = tuple(map(int, rospy.get_param('starting_pose', (0, 0))))
        self.goal = tuple(map(int, rospy.get_param('goal_pose', (0, 0))))
        self.obstacles = {(int(x), int(y)) for x, y in zip(xs, ys)}
        xs_all = [p[0] for p in self.obstacles] + [self.start[0], self.goal[0]]
        ys_all = [p[1] for p in self.obstacles] + [self.start[1], self.goal[1]]
        self.xmin, self.xmax = min(xs_all) - 1, max(xs_all) + 1
        self.ymin, self.ymax = min(ys_all) - 1, max(ys_all) + 1
        raw = self._astar()
        self.raw_path = raw
        self.smooth_path = self._catmull_rom(raw, steps=20)
        self._publish_raw_path()
        self.marker = self._init_marker()

    def _heur(self, a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])

    def _neighbors(self, node):
        x, y = node
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = x + dx, y + dy
            if not (self.xmin <= nx <= self.xmax and self.ymin <= ny <= self.ymax):
                continue
            if (nx, ny) in self.obstacles:
                continue
            if dx != 0 and dy != 0:
                if (x+dx, y) in self.obstacles or (x, y+dy) in self.obstacles:
                    continue
            yield (nx, ny), np.hypot(dx, dy)

    def _astar(self):
        open_list = [(self.start, 0.0, self._heur(self.start, self.goal), None)]
        closed = {}
        while open_list:
            i = min(range(len(open_list)), key=lambda i: open_list[i][2])
            node, g, f, parent = open_list.pop(i)
            if node in closed and closed[node] <= g:
                continue
            closed[node] = g
            if node == self.goal:
                path, cur = [], (node, g, f, parent)
                while cur:
                    path.append(cur[0])
                    cur = cur[3]
                return path[::-1]
            for nbr, cost in self._neighbors(node):
                open_list.append((nbr, g + cost, g + cost + self._heur(nbr, self.goal), (node, g, f, parent)))
        return []

    def _catmull_rom(self, path, steps=10):
        if len(path) < 2:
            return path
        sm = []
        for i in range(len(path) - 1):
            p0 = path[i - 1] if i > 0 else path[i]
            p1, p2 = path[i], path[i + 1]
            p3 = path[i + 2] if i + 2 < len(path) else path[i + 1]
            for t in np.linspace(0, 1, steps, endpoint=False):
                x = 0.5*(2*p1[0] + (-p0[0]+p2[0])*t + (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t**2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t**3)
                y = 0.5*(2*p1[1] + (-p0[1]+p2[1])*t + (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t**2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t**3)
                sm.append((x, y))
        sm.append(path[-1])
        return sm

    def _publish_raw_path(self):
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        for x, y in self.raw_path:
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = x
            ps.pose.position.y = y
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _init_marker(self):
        m = Marker()
        m.header.frame_id = "world"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r = 1.0
        m.color.a = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        return m

    def run(self):
        for x, y in self.smooth_path:
            if rospy.is_shutdown():
                break
            self.marker.header.stamp = rospy.Time.now()
            self.marker.pose.position.x = x
            self.marker.pose.position.y = y
            self.pose_pub.publish(self.marker)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        PathGenerator().run()
    except rospy.ROSInterruptException:
        pass
