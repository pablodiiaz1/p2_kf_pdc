import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        #odometry
        self.odom_subscription = self.create_subscription(
            
        )
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1

        self.kf = KalmanFilter(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        #Set the initial pose
        if not self.initial_pose:
            self.initial_pose = odom_to_pose2D(msg)
        #Get and normalize the pose
        current_pose = odom_to_pose2D(msg)
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, current_pose))

        if self.first prediction_done:
            curr_time_secs = self.get_clock().now().nanoseconds/1e9
            z = self.odom_simulator.add_drift(self.normalized_pose, curr_time_secs)
            mu, Sigma = self.kf.update(z)

            #visualization
            self.Visualizer.update(self.normalized_pose, mu, Sigma, step='update')
            self.publish_estimated_pose(mu)
            self.publish_real_pose(self.normalized_pose)
            self.get_logger().info(f"Updated State: {mu}")
            self.get_logger().info(f"Measured Pose: {self.normalized_pose}")

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
