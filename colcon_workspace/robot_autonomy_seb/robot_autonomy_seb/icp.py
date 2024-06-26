import math
import time

import numpy as np
import rclpy
import transforms3d as t3d
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .utils import rotation_to_angle, scan_to_points, get_transform, transform
from nav_msgs.msg import Odometry

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ICP:
    """
    This class contains the function to execute the ICP algorithm. It is seperated from the
    ICPNode class to make it easier to test (see test/test_icp.py).
    """

    def __init__(self, logger=None, clock=None):
        """
        Create the icp object. It creates some var that will be used later.

        :param logger:
        :param clock:
        """

        self.__logger__ = logger
        self.__clock__ = clock
        self.sources = None

    def clock(self):
        """
        :return: Get the current time in milliseconds (either using ROS or time package)
        """
        if self.__clock__ is not None:
            return self.__clock__.now().nanoseconds
        return time.time()

    def log(self, string):
        """
        :param string: the text to be logged
        :return: Log the given string either to the ROS logger or using print
        """

        if self.__logger__ is not None:
            self.__logger__.info(string) / 1000_000_000
        else:
            print(string)

    def execute_icp(self, targets):
        """
        Compute the transformation between the previous supplied targets and the new one

        :param targets: the new point cloud 2xN where N is the number of points in the point cloud
        :return: None the first time, then a transformation matrix 3x3
        """
        T = None

        if self.sources is not None:
            T = np.eye(3)
            b = 0
            for i in range(20):
                # find the closet point to each target point from the source (with the transformation of the previous step)
                targets_matched, sources_matched = self.reorder(transform(T, self.sources), targets)
                if np.mean(np.linalg.norm(sources_matched - targets_matched, axis=0)) < 0.001:
                    break
                # compute the transformation
                T = np.dot(T, self.compute_transformation(sources_matched, targets_matched))

            # self.log(f"mean dist {abs(b - np.mean(np.linalg.norm(sources_matched - targets_matched, axis=0)))}")
        self.sources = targets
        return T

    def reorder(self, sources, targets, max_dist=0.5):
        """
        Computes an array sources_matched where sources_matched[:, i] is the closest point to targets[:, i] from self.sources

        :param sources: the old point cloud
        :param targets: the new point cloud
        :param max_dist: the distance at which a correspondence is rejected (in meters)
        :return: two point cloud targets_matched and sources_matched, where targets_matched[:, i] is the closest point
                    to sources_matched[:, i]
        """
        targets_matched = np.ones(targets.shape, dtype=np.float32) * np.inf
        sources_matched = np.ones(targets.shape, dtype=np.float32) * np.inf
        idx = 0

        for i in range(targets.shape[1]):
            target = targets[:, i:i + 1]

            distances = np.linalg.norm(sources - target, axis=0)
            m = np.argmin(distances)
            if distances[m] < max_dist:
                targets_matched[:, i] = target[:, 0]
                sources_matched[:, i] = sources[:, m]
                idx += 1

        return targets_matched[:, targets_matched[0, :] < np.inf], sources_matched[:, sources_matched[0, :] < np.inf]

    def compute_transformation(self, source_matched, target_matched):
        """
        Use the svd algorithm to compute the transformation matrix

        :param source_matched: the source point cloud 2xN where N is the number of points in the point cloud
        :param target_matched: the target point cloud 2xN where N is the number of points in the point cloud
        :return: the transformation matrix to transform the source point cloud into target point cloud
        """

        centroid_source = source_matched.mean(axis=1)[:, np.newaxis]
        centered_source = source_matched - centroid_source
        centroid_target = target_matched.mean(axis=1)[:, np.newaxis]
        centered_target = target_matched - centroid_target

        H = np.dot(centered_target, centered_source.T) / centered_source.shape[1]
        U, S, Vh = np.linalg.svd(H)
        R = np.dot(U, Vh)
        t = centroid_target - np.dot(R, centroid_source)

        T = np.zeros((3, 3), dtype=np.float32)
        T[:2, :2] = R
        T[:2, 2] = t[:, 0]
        T[2, 2] = 1
        return T


class ICPNode(Node):
    """
    This is a small class to interface ICP to ROS
    """

    def __init__(self):
        super().__init__('homemade_icp')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.publisher = self.create_publisher(Odometry, 'lidar_odom', 10)

        self.icp = ICP()
        self.last_time = self.get_clock().now().nanoseconds / 1_000_000_000
        self.pose_x = 0.
        self.pose_y = 0.
        self.angle = 0.

    def listener_callback(self, scan):
        """
        Callback when new lidar measurement is received
            :param scan: the data (see https://docs.ros.org/en/latest/api/sensor_msgs/html/msg/LaserScan.html)
        """
        t = scan.header.stamp.nanosec / 1_000_000_000 + scan.header.stamp.sec
        dt = t - self.last_time
        if abs(dt) < 0.00001:
            self.get_logger().warn(f"Time delta is too small ({dt}). Skipping this scan.")
            return

        targets = scan_to_points(scan)
        T = self.icp.execute_icp(targets)
        if T is None:
            if self.pose_x != 0. or self.pose_y != 0. or self.angle != 0.:
                self.get_logger().warn(f"Transform is None.")
            return

        dx, dy = -T[0, 2], T[1, 2]
        dx_in_dom = dx * math.cos(self.angle) - dy * math.sin(self.angle)
        dy_in_dom = dx * math.sin(self.angle) + dy * math.cos(self.angle)
        da = -rotation_to_angle(T[:2, :2])

        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Twist of odom
        odom.twist.twist.linear.x = dx_in_dom / dt
        odom.twist.twist.linear.y = dy_in_dom / dt
        odom.twist.twist.linear.z = 0.
        odom.twist.twist.angular.z = da / dt

        self.pose_x += dx_in_dom
        self.pose_y += dy_in_dom
        self.angle += da

        # Pose of odom
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.

        q = t3d.euler.euler2quat(0, 0, self.angle)
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]

        self.publisher.publish(odom)

        self.last_time = t


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ICPNode()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
