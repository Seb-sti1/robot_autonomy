import numpy as np
import rclpy
import transforms3d as t3d
from tf2_ros import TransformException


def rotation_to_angle(R):
    return np.arctan2(-R[0, 1], R[0, 0])


def ranges_to_points(r, a):
    return np.array([r * np.cos(a), r * np.sin(a)])


def scan_to_points(scan):
    """
    :param scan: https://docs.ros.org/en/latest/api/sensor_msgs/html/msg/LaserScan.html
    :return: 2xN numpy array, R[0, :] are the x values and R[1, :] are the y values
    """

    ids = [i for i, r in enumerate(scan.ranges) if scan.range_min <= r <= scan.range_max]
    return ranges_to_points(np.array(scan.ranges)[ids],
                            np.linspace(scan.angle_min,
                                        scan.angle_max,
                                        int((scan.angle_max - scan.angle_min) // scan.angle_increment) + 1,
                                        dtype=np.float32)[ids])


def transform(T, points):
    return np.matmul(T[:2, :2], points) + T[:2, 2:3]


def get_transform(node, from_frame, to_frame):
    try:
        map_to_lidar = node.tf_buffer.lookup_transform(
            from_frame,
            to_frame,
            rclpy.time.Time())
    except TransformException as ex:
        node.get_logger().debug(
            f'Could not transform {from_frame} to {to_frame}: {ex}')
        return

    T = np.eye(3)
    T[:2, :2] = t3d.quaternions.quat2mat([map_to_lidar.transform.rotation.w,
                                          map_to_lidar.transform.rotation.x,
                                          map_to_lidar.transform.rotation.y,
                                          map_to_lidar.transform.rotation.z])[:2, :2]
    T[:2, 2] = [map_to_lidar.transform.translation.x, map_to_lidar.transform.translation.y]
    T[2, 2] = 1

    return T