import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from .utils import get_transform, transform, scan_to_points


def bresenham(a, b, func):
    """
    Will call func for any points on the line between a and b.
    See https://en.wikipedia.org/wiki/Bresenham's_line_algorithm#Algorithm_for_integer_arithmetic
    :param a: Tuple[int] coords
    :param b: Tuple[int] coords
    :param func: Callable which takes coords as input
    """
    x_start, y_start = a
    x_end, y_end = b

    dx = x_end - x_start
    dy = y_end - y_start

    is_steep = abs(dy) > abs(dx)
    if is_steep:
        x_start, y_start = y_start, x_start
        x_end, y_end = y_end, x_end

    if x_start > x_end:
        x_start, x_end = x_end, x_start
        y_start, y_end = y_end, y_start

    dx = x_end - x_start
    dy = y_end - y_start

    error = int(dx / 2.0)

    y_step = 1 if y_start < y_end else -1
    y = y_start
    for x in range(x_start, x_end + 1):
        coord = [y, x] if is_steep else [x, y]
        if not func(coord[0], coord[1], a, b):
            return  # if (coord[0], coord[1]) isn't in the map stop the algorithm
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx


class MapNode(Node):
    def __init__(self):
        super().__init__('homemap_map')

        # subscribe to lidar and robot position
        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # see definition https://docs.ros.org/en/latest/api/nav_msgs/html/msg/OccupancyGrid.html
        self.map = OccupancyGrid()
        self.map.info.resolution = 0.05
        self.map.info.width = 450
        self.map.info.height = 200
        # put the map at the center
        self.map.info.origin.position.x = - self.map.info.resolution * self.map.info.width / 2
        self.map.info.origin.position.y = - self.map.info.resolution * self.map.info.height / 2
        self.map.info.origin.orientation.x = 0.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 1.0
        self.map.header.frame_id = 'map'

        # TODO(not important) stores odds? instead of p * 100
        self.raw_map = -np.ones((self.map.info.height, self.map.info.width), dtype=int)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latching_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,  # Feel wrong
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, '/lidar_map', self.latching_qos)

        self.robot_pose = None

    def from_world_to_coordinates(self, pose):
        coord = (int((pose[1, 0] - self.map.info.origin.position.y) / self.map.info.resolution),
                 int((pose[0, 0] - self.map.info.origin.position.x) / self.map.info.resolution))
        return coord

    def scan_callback(self, scan):
        """
        Called when a scan is received.
        :param scan: the data
        """
        if self.robot_pose is None:
            return

        # find the transformation between the lidar and the map
        map_to_lidar = get_transform(self,
                                     self.map.header.frame_id,
                                     scan.header.frame_id)
        if map_to_lidar is None:
            return

        # TODO(not important) use inf point to color only white ray
        # get the points and transform them
        points = transform(map_to_lidar, scan_to_points(scan))

        # update map
        starting_coord = self.from_world_to_coordinates(self.robot_pose)
        if not (0 <= starting_coord[0] < self.map.info.height and 0 <= starting_coord[1] < self.map.info.width):
            return

        def update_map(x, y, a, b):
            """
            Update the value of the map at a specific point if (x, y) is in the map

            :param x: the x coordinate of the point to update
            :param y: the y coordinate of the point to update
            :param a: the start of the line
            :param b: the end of the line
            :result: return if (x, y) is in the map
            """
            if not (0 <= x < self.map.info.height and 0 <= y < self.map.info.width):
                return False

            # use a normal distribution for the probability of having an object along the path
            # clamp value to make sure that 1/(1-p) exists
            p = max(min(0.4 * math.exp(- ((x - b[0]) ** 2 + (y - b[1]) ** 2) / 2) + 0.3, 0.99), 0.01)

            if self.raw_map[x, y] == -1:  # unknown value
                self.raw_map[x, y] = int(p * 100)
            else:
                # See https://en.wikipedia.org/wiki/Odds
                # See https://perso.ensta-paris.fr/%7Emanzaner/Cours/ROB201/ROB201-Slides03.pdf slide 11
                p_prev = max(min(self.raw_map[x, y] / 100, 0.99), 0.01)
                new_odd = p * p_prev / (1 - p) / (1 - p_prev)
                self.raw_map[x, y] = int(new_odd / (1 + new_odd) * 100)
            return True

        for i in range(points.shape[1]):
            coord = self.from_world_to_coordinates(points[:, i:i + 1])
            bresenham(starting_coord, coord, update_map)

        self.map.header.stamp = scan.header.stamp
        self.map.data = self.raw_map.reshape((self.map.info.height * self.map.info.width)).tolist()
        self.pub.publish(self.map)

    def odom_callback(self, odom):
        """
        Called when an Odometry message is received. It keeps the self.robot_pose up-to-date.
        :param odom: odometry message
        """
        map_to_odom = get_transform(self,
                                    self.map.header.frame_id,
                                    odom.header.frame_id)
        if map_to_odom is None:
            return

        self.robot_pose = transform(map_to_odom, np.array([[odom.pose.pose.position.x,
                                                            odom.pose.pose.position.y]]).T)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MapNode()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
