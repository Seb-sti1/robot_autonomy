from typing import Dict

import numpy as np
import math as ma
import random

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .utils import get_transform, transform

coords = tuple[int, int]


def __from_np_to_tuple__(nparray):
    return nparray[0, 0], nparray[1, 0]


class RRTTree:

    def __init__(self, init_pose, init_visible):
        self.edges: Dict[coords, list[coords]] = {__from_np_to_tuple__(init_pose): []}
        self.gain: Dict[coords, float] = {__from_np_to_tuple__(init_pose): init_visible}

    def get_vertices(self) -> "np.array":
        vertices = list(self.edges.keys())
        result = np.zeros((2, len(vertices)))

        for i, vertex in enumerate(vertices):
            result[0, i] = vertex[0]
            result[1, i] = vertex[1]

        return result

    def connect_new_point(self, a, b, visible):
        """
        Connect a to b. a is an existing point, b is a new one

        param a: coords
        param b: coords
        param visible: the visible score in the gain
        """

        self.edges[__from_np_to_tuple__(a)].append(__from_np_to_tuple__(b))
        self.edges[__from_np_to_tuple__(b)] = []
        self.gain[__from_np_to_tuple__(b)] = self.gain[__from_np_to_tuple__(a)] + visible

        return self.gain[__from_np_to_tuple__(b)]

    def print(self, logger):
        for ele in self.edges:
            logger(f"{ele} ({self.gain[ele]:.1f}): {';'.join([f'{name}' for name in self.edges[ele]])}")


class NBVNode(Node):
    """
    The node executing the Next Best View algorithm to find the next point to go to
    """

    def __init__(self):
        super().__init__("homemade_nbv")
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.listener_callback, 10)
        self.odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.marker = self.create_publisher(MarkerArray, "/rrt_tree", 10)
        self.nav = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.nav.setInitialPose(initial_pose)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.raw_map = None
        self.rrt_d = 0.7  # m
        self.rrt_Nmax = 15
        self.nbv_lambda = 1
        self.lidar_max_range = 3.5  # m

        self.robot_pose = None

        self.should_send_next_goal = False

        self.get_logger().info("Waiting for Nav2 server")
        self.nav.waitUntilNav2Active(localizer="bt_navigator")
        self.get_logger().info("Nav2 server ready")
        self.should_send_next_goal = True
        self.timer = self.create_timer(10, self.timer_callback)

    def timer_callback(self):
        if self.map is None or self.robot_pose is None or not self.should_send_next_goal:
            return
        self.should_send_next_goal = False
        self.send_goal()
        self.should_send_next_goal = True

    def send_goal(self):
        self.raw_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        point = self.find_next_best_view()
        self.get_logger().info(f"Next best view suggest to go at {point.T}")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = point[0, 0]
        goal_pose.pose.position.y = point[1, 0]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                self.nav.cancelTask()

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')

    def from_world_to_coordinates(self, pose):
        coord = (
            int(
                (pose[1, 0] - self.map.info.origin.position.y)
                / self.map.info.resolution
            ),
            int(
                (pose[0, 0] - self.map.info.origin.position.x)
                / self.map.info.resolution
            ),
        )

        if 0 <= coord[0] < self.map.info.width and 0 <= coord[1] < self.map.info.height:
            return np.array([coord]).T
        return None

    def from_coordinates_to_world(self, coord):
        return np.array(
            [
                [
                    coord[1, 0] * self.map.info.resolution
                    + self.map.info.origin.position.x,
                    coord[0, 0] * self.map.info.resolution
                    + self.map.info.origin.position.y,
                ]
            ]
        ).T

    def visible_score(self, a, b):
        dist = np.linalg.norm(b - a) * self.map.info.resolution

        radius = self.lidar_max_range / self.map.info.resolution
        x, y = np.indices((self.map.info.height, self.map.info.width))
        in_range = self.raw_map[np.power(x, 2) + np.power(y, 2) <= ma.pow(radius, 2)]

        count_unknown_visible_cell = np.sum(in_range == -1)

        return count_unknown_visible_cell * ma.exp(-self.nbv_lambda * dist)

    def rrt_next_point(self, G):
        possible = np.where(self.raw_map != -1)
        idx = random.randrange(len(possible[0]))
        q_rand = np.array([[possible[0][idx], possible[1][idx]]]).T

        distances = np.linalg.norm(G.get_vertices() - q_rand, axis=0)
        m = np.argmin(distances)
        q_nearest = G.get_vertices()[:, m: m + 1]

        q_new = (
                        1 / np.linalg.norm(q_rand - q_nearest) * (
                        q_rand - q_nearest) * self.rrt_d / self.map.info.resolution
                ).astype(np.int) + q_nearest

        return q_new, G.connect_new_point(
            q_nearest, q_new, self.visible_score(q_nearest, q_new)
        )

    def find_next_best_view(self):
        """
        Find the next best view coordinates to go to continue the exploration
        """
        candidate = None
        n = 0
        G = RRTTree(self.from_world_to_coordinates(self.robot_pose), 0)

        while n < self.rrt_Nmax:
            q_new, g = self.rrt_next_point(G)

            if candidate is None or candidate[0] < g:
                candidate = (g, q_new)
            n += 1

        self.publish_marker(G)
        return self.from_coordinates_to_world(candidate[1])

    def create_marker(self, uid, type, x, y, size, is_blue):
        marker = Marker()
        marker.action = Marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = uid
        marker.type = type

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)

        marker.color.r = 0.0
        marker.color.g = 0.0 if is_blue else 1.0
        marker.color.b = 1.0 if is_blue else 0.0
        marker.color.a = 1.0

        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = 0.005

        return marker

    def publish_marker(self, G, marker_size=0.3, marker_min_size=0.001):
        # remove all marker
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(marker)
        self.marker.publish(markers)

        # create new one
        markers = MarkerArray()
        max_score = max([G.gain[k] for k in G.gain])

        lines = self.create_marker(0, Marker.LINE_LIST, 0, 0, 0.01, False)
        uid = 1

        for ele in G.edges:
            pose = self.from_coordinates_to_world(np.array([[ele[0], ele[1]]]).T)
            markers.markers.append(self.create_marker(uid, Marker.CYLINDER, pose[0, 0], pose[1, 0],
                                                      max(marker_size * G.gain[ele] / max_score, marker_min_size),
                                                      True))
            uid += 1

            p_ele = Point()
            p_ele.x, p_ele.y, p_ele.z = pose[0, 0], pose[1, 0], 0.
            for c in G.edges[ele]:
                pose_c = self.from_coordinates_to_world(np.array([[c[0], c[1]]]).T)
                p_c = Point()
                p_c.x, p_c.y, p_c.z = pose_c[0, 0], pose_c[1, 0], 0.

                lines.points.append(p_ele)
                lines.points.append(p_c)

        markers.markers.append(lines)
        self.marker.publish(markers)

    def listener_callback(self, data):
        """
        Callback when new map is received
        :param data: the data
        """
        self.map = data

    def odom_callback(self, odom):
        """
        Called when an Odometry message is received. It keeps the self.robot_pose up-to-date.
        :param odom: odometry message
        """
        if self.map == None:
            return

        map_to_odom = get_transform(
            self, self.map.header.frame_id, odom.header.frame_id
        )
        if map_to_odom is None:
            return

        self.robot_pose = transform(
            map_to_odom,
            np.array([[odom.pose.pose.position.x, odom.pose.pose.position.y]]).T,
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = NBVNode()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
