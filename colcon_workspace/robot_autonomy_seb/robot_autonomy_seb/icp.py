import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def rotation(theta, points):
    return np.matmul(np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]), points)


def gradient(theta, last_points, current_points):
    # [(Xi, Yi) ...]
    rotated = rotation(theta, current_points)
    # [(dXi/dtheta, dYi/dtheta) ... ]
    derivative = np.matmul(np.array([[-np.sin(theta), -np.cos(theta)], [np.cos(theta), -np.sin(theta)]]),
                           current_points)

    return 2 * np.sum(np.multiply(rotated - last_points, derivative))


class ICP(Node):

    def __init__(self):
        super().__init__('homemade_icp')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, 'lidar_twist', 10)

        self.last_points = None
        self.last_time = None

    def listener_callback(self, scan):

        angles = np.array([scan.angle_min + i * scan.angle_increment for i, d in enumerate(scan.ranges)
                           if scan.range_min <= d <= scan.range_max])[:10]
        ranges = np.array([d for d in scan.ranges
                           if scan.range_min <= d <= scan.range_max])[:10]
        current_points = np.append([np.multiply(ranges, np.cos(angles))], [np.multiply(ranges, np.sin(angles))], axis=0)

        if self.last_points is not None:
            center_translation = np.mean(self.last_points, axis=1) - np.mean(current_points, axis=1)
            shifted_points = current_points + center_translation[:, np.newaxis]

            # last_points = [ (bar_xi, bar_yi) ... ]
            # shifted_points = [ (xi, yi) ...]
            # rotation(theta, shifted_points) =  [ (Xi, Yi) ...]
            #
            # cost = sum_i [ (Xi - bar_xi)**2 + (Yi - bar_Yi)**2 ]
            #
            # dc/dtheta = sum_i [ 2 dXi/dtheta * (Xi - bar_xi) + 2 dYi/dtheta * (Yi - bar_yi) ]
            # dXi/dtheta = -xi sin theta - yi cos theta
            # dYi/dtheta = xi cos theta - yi sin theta
            theta = 0
            rate = 0.01
            grad = gradient(theta, self.last_points, shifted_points)
            i = 0

            # Need tweaking
            while abs(grad) > 0.1 and i < 1000:
                theta = theta + rate * grad
                i += 1

            dt = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9

            twist = Twist()
            twist.linear.x = center_translation[0]/dt
            twist.linear.y = center_translation[1]/dt
            twist.linear.z = 0.
            twist.angular.z = theta/dt

            self.publisher.publish(twist)

        self.last_points = current_points
        self.last_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ICP()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
