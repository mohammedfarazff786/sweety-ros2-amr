#!/usr/bin/env python3
"""
Obstacle Avoidance Node with DWA Planner Implementation.

This node implements the Dynamic Window Approach (DWA) for local path planning
and obstacle avoidance in the Sweety AMR.
"""

import math
import numpy as np
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class DWAPlanner:
    """Dynamic Window Approach (DWA) planner for local path planning."""

    def __init__(
        self,
        max_speed: float = 1.0,
        min_speed: float = 0.0,
        max_angular_speed: float = 1.5,
        acceleration: float = 0.5,
        angular_acceleration: float = 1.0,
        predict_time: float = 1.0,
        dt: float = 0.1,
        velocity_resolution: float = 0.05,
        angular_resolution: float = 0.05,
        cost_heading_weight: float = 0.15,
        cost_obstacle_weight: float = 0.1,
        cost_velocity_weight: float = 0.75,
        obstacle_distance_threshold: float = 0.3,
    ):
        """Initialize DWA planner parameters.

        Args:
            max_speed: Maximum linear velocity (m/s)
            min_speed: Minimum linear velocity (m/s)
            max_angular_speed: Maximum angular velocity (rad/s)
            acceleration: Linear acceleration limit (m/s²)
            angular_acceleration: Angular acceleration limit (rad/s²)
            predict_time: Prediction horizon (s)
            dt: Time step for simulation (s)
            velocity_resolution: Linear velocity resolution (m/s)
            angular_resolution: Angular velocity resolution (rad/s)
            cost_heading_weight: Weight for heading cost
            cost_obstacle_weight: Weight for obstacle cost
            cost_velocity_weight: Weight for velocity cost
            obstacle_distance_threshold: Minimum distance to obstacle (m)
        """
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_angular_speed = max_angular_speed
        self.acceleration = acceleration
        self.angular_acceleration = angular_acceleration
        self.predict_time = predict_time
        self.dt = dt
        self.velocity_resolution = velocity_resolution
        self.angular_resolution = angular_resolution
        self.cost_heading_weight = cost_heading_weight
        self.cost_obstacle_weight = cost_obstacle_weight
        self.cost_velocity_weight = cost_velocity_weight
        self.obstacle_distance_threshold = obstacle_distance_threshold

        self.current_speed = 0.0
        self.current_angular_speed = 0.0
        self.goal_angle = 0.0
        self.obstacle_ranges = []

    def update_odometry(self, linear_velocity: float, angular_velocity: float) -> None:
        """Update current velocity from odometry.

        Args:
            linear_velocity: Current linear velocity (m/s)
            angular_velocity: Current angular velocity (rad/s)
        """
        self.current_speed = linear_velocity
        self.current_angular_speed = angular_velocity

    def update_goal(self, goal_angle: float) -> None:
        """Update goal direction.

        Args:
            goal_angle: Goal angle relative to robot (rad)
        """
        self.goal_angle = goal_angle

    def update_obstacles(self, ranges: List[float], angle_min: float, angle_max: float) -> None:
        """Update obstacle information from laser scan.

        Args:
            ranges: List of distance measurements
            angle_min: Minimum scan angle (rad)
            angle_max: Maximum scan angle (rad)
        """
        self.obstacle_ranges = ranges
        self.angle_min = angle_min
        self.angle_max = angle_max

    def calculate_dynamic_window(
        self, current_speed: float, current_angular_speed: float
    ) -> Tuple[float, float, float, float]:
        """Calculate dynamic window based on acceleration constraints.

        Args:
            current_speed: Current linear velocity (m/s)
            current_angular_speed: Current angular velocity (rad/s)

        Returns:
            Tuple of (min_speed, max_speed, min_angular_speed, max_angular_speed)
        """
        # Apply acceleration constraints
        max_speed = min(
            self.max_speed,
            current_speed + self.acceleration * self.dt,
        )
        min_speed = max(
            self.min_speed,
            current_speed - self.acceleration * self.dt,
        )

        max_angular_speed = min(
            self.max_angular_speed,
            current_angular_speed + self.angular_acceleration * self.dt,
        )
        min_angular_speed = max(
            -self.max_angular_speed,
            current_angular_speed - self.angular_acceleration * self.dt,
        )

        return min_speed, max_speed, min_angular_speed, max_angular_speed

    def predict_trajectory(
        self, speed: float, angular_speed: float, current_pose: Tuple[float, float, float]
    ) -> np.ndarray:
        """Predict trajectory for given velocities.

        Args:
            speed: Linear velocity (m/s)
            angular_speed: Angular velocity (rad/s)
            current_pose: Current pose (x, y, theta)

        Returns:
            Array of predicted trajectory points
        """
        trajectory = np.array([current_pose])
        x, y, theta = current_pose

        for _ in np.arange(0, self.predict_time, self.dt):
            # Update pose based on motion model
            if abs(angular_speed) < 1e-6:
                x += speed * math.cos(theta) * self.dt
                y += speed * math.sin(theta) * self.dt
            else:
                # Differential drive kinematic model
                radius = speed / angular_speed
                x += radius * (math.sin(theta + angular_speed * self.dt) - math.sin(theta))
                y += radius * (-math.cos(theta + angular_speed * self.dt) + math.cos(theta))
                theta += angular_speed * self.dt

            trajectory = np.vstack([trajectory, [x, y, theta]])

        return trajectory

    def calculate_obstacle_cost(
        self, trajectory: np.ndarray, obstacle_distance_threshold: Optional[float] = None
    ) -> float:
        """Calculate cost based on proximity to obstacles.

        Args:
            trajectory: Predicted trajectory
            obstacle_distance_threshold: Minimum safe distance to obstacles

        Returns:
            Obstacle cost (0.0 to infinity)
        """
        if obstacle_distance_threshold is None:
            obstacle_distance_threshold = self.obstacle_distance_threshold

        if not self.obstacle_ranges:
            return 0.0

        # Filter out invalid range values
        valid_ranges = [
            r for r in self.obstacle_ranges if not math.isinf(r) and not math.isnan(r) and r > 0
        ]

        if not valid_ranges:
            return 0.0

        min_distance = min(valid_ranges)

        # If too close to obstacle, return high cost
        if min_distance < obstacle_distance_threshold:
            return float('inf')

        # Cost increases as robot gets closer to obstacles
        obstacle_cost = 1.0 / (min_distance + 0.1)

        return obstacle_cost

    def calculate_heading_cost(self, final_pose: Tuple[float, float, float]) -> float:
        """Calculate cost based on heading difference from goal.

        Args:
            final_pose: Final pose after trajectory prediction (x, y, theta)

        Returns:
            Heading cost (0.0 to 1.0)
        """
        _, _, theta = final_pose

        # Calculate angle difference
        angle_diff = abs(self.goal_angle - theta)
        # Normalize to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        angle_diff = abs(angle_diff)

        # Cost is proportional to angle difference
        heading_cost = angle_diff / math.pi

        return heading_cost

    def calculate_velocity_cost(self, speed: float) -> float:
        """Calculate cost to encourage higher velocities.

        Args:
            speed: Linear velocity (m/s)

        Returns:
            Velocity cost (0.0 to 1.0)
        """
        if self.max_speed <= 0:
            return 0.0

        # Cost decreases as velocity increases
        velocity_cost = 1.0 - (speed / self.max_speed)

        return velocity_cost

    def calculate_total_cost(
        self,
        trajectory: np.ndarray,
        speed: float,
        obstacle_cost: float,
        heading_cost: float,
        velocity_cost: float,
    ) -> float:
        """Calculate weighted total cost.

        Args:
            trajectory: Predicted trajectory
            speed: Linear velocity
            obstacle_cost: Obstacle-based cost
            heading_cost: Heading-based cost
            velocity_cost: Velocity-based cost

        Returns:
            Total weighted cost
        """
        total_cost = (
            self.cost_obstacle_weight * obstacle_cost
            + self.cost_heading_weight * heading_cost
            + self.cost_velocity_weight * velocity_cost
        )

        return total_cost

    def plan(
        self, current_pose: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Plan next velocity command using DWA.

        Args:
            current_pose: Current robot pose (x, y, theta)

        Returns:
            Tuple of (linear_velocity, angular_velocity, min_cost)
        """
        # Calculate dynamic window
        min_speed, max_speed, min_angular_speed, max_angular_speed = self.calculate_dynamic_window(
            self.current_speed, self.current_angular_speed
        )

        best_cost = float('inf')
        best_speed = 0.0
        best_angular_speed = 0.0

        # Generate candidate velocities
        speed_range = np.arange(min_speed, max_speed + self.velocity_resolution, self.velocity_resolution)
        angular_range = np.arange(
            min_angular_speed,
            max_angular_speed + self.angular_resolution,
            self.angular_resolution,
        )

        for speed in speed_range:
            for angular_speed in angular_range:
                # Predict trajectory
                trajectory = self.predict_trajectory(speed, angular_speed, current_pose)

                # Calculate costs
                obstacle_cost = self.calculate_obstacle_cost(trajectory)

                # Skip if collision
                if math.isinf(obstacle_cost):
                    continue

                final_pose = tuple(trajectory[-1])
                heading_cost = self.calculate_heading_cost(final_pose)
                velocity_cost = self.calculate_velocity_cost(speed)

                # Calculate total cost
                total_cost = self.calculate_total_cost(
                    trajectory, speed, obstacle_cost, heading_cost, velocity_cost
                )

                # Update best velocity
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_speed = speed
                    best_angular_speed = angular_speed

        return best_speed, best_angular_speed, best_cost


class ObstacleAvoidanceNode(Node):
    """ROS 2 Node for obstacle avoidance using DWA planner."""

    def __init__(self):
        """Initialize the obstacle avoidance node."""
        super().__init__('obstacle_avoidance_node')

        # Declare parameters
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('acceleration', 0.5)
        self.declare_parameter('angular_acceleration', 1.0)
        self.declare_parameter('predict_time', 1.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('velocity_resolution', 0.05)
        self.declare_parameter('angular_resolution', 0.05)
        self.declare_parameter('cost_heading_weight', 0.15)
        self.declare_parameter('cost_obstacle_weight', 0.1)
        self.declare_parameter('cost_velocity_weight', 0.75)
        self.declare_parameter('obstacle_distance_threshold', 0.3)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('laser_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Get parameters
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        max_angular_speed = self.get_parameter('max_angular_speed').value
        acceleration = self.get_parameter('acceleration').value
        angular_acceleration = self.get_parameter('angular_acceleration').value
        predict_time = self.get_parameter('predict_time').value
        dt = self.get_parameter('dt').value
        velocity_resolution = self.get_parameter('velocity_resolution').value
        angular_resolution = self.get_parameter('angular_resolution').value
        cost_heading_weight = self.get_parameter('cost_heading_weight').value
        cost_obstacle_weight = self.get_parameter('cost_obstacle_weight').value
        cost_velocity_weight = self.get_parameter('cost_velocity_weight').value
        obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value
        control_frequency = self.get_parameter('control_frequency').value
        laser_topic = self.get_parameter('laser_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Initialize DWA planner
        self.dwa_planner = DWAPlanner(
            max_speed=max_speed,
            min_speed=min_speed,
            max_angular_speed=max_angular_speed,
            acceleration=acceleration,
            angular_acceleration=angular_acceleration,
            predict_time=predict_time,
            dt=dt,
            velocity_resolution=velocity_resolution,
            angular_resolution=angular_resolution,
            cost_heading_weight=cost_heading_weight,
            cost_obstacle_weight=cost_obstacle_weight,
            cost_velocity_weight=cost_velocity_weight,
            obstacle_distance_threshold=obstacle_distance_threshold,
        )

        # Current state
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.goal_angle = 0.0
        self.enabled = False

        # QoS profile
        qos = QoSProfile(depth=10)

        # Subscriptions
        self.laser_sub = self.create_subscription(LaserScan, laser_topic, self.laser_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)
        self.goal_sub = self.create_subscription(Twist, goal_topic, self.goal_callback, qos)

        # Publications
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos)

        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0 / control_frequency, self.control_callback
        )

        self.get_logger().info('Obstacle avoidance node initialized')

    def laser_callback(self, msg: LaserScan) -> None:
        """Handle laser scan messages.

        Args:
            msg: LaserScan message
        """
        self.dwa_planner.update_obstacles(
            list(msg.ranges), msg.angle_min, msg.angle_max
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Handle odometry messages.

        Args:
            msg: Odometry message
        """
        # Extract position
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Extract velocity
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        # Convert quaternion to yaw angle
        theta = self.quaternion_to_yaw(orientation)

        self.current_pose = (position.x, position.y, theta)
        self.dwa_planner.update_odometry(linear_velocity, angular_velocity)

    def goal_callback(self, msg: Twist) -> None:
        """Handle goal/desired velocity messages.

        Args:
            msg: Twist message containing goal velocity
        """
        # Extract goal angle from twist message
        self.goal_angle = math.atan2(msg.linear.y, msg.linear.x)
        self.dwa_planner.update_goal(self.goal_angle)

    def control_callback(self) -> None:
        """Execute control loop and publish velocity commands."""
        if not self.enabled:
            # Publish zero velocity
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        try:
            # Plan next velocity
            linear_velocity, angular_velocity, cost = self.dwa_planner.plan(self.current_pose)

            # Publish command
            cmd_vel = Twist()
            cmd_vel.linear.x = float(linear_velocity)
            cmd_vel.angular.z = float(angular_velocity)

            self.cmd_vel_pub.publish(cmd_vel)

            self.get_logger().debug(
                f'Published velocity: v={linear_velocity:.2f}, w={angular_velocity:.2f}, cost={cost:.4f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
            # Publish zero velocity on error
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)

    @staticmethod
    def quaternion_to_yaw(quaternion) -> float:
        """Convert quaternion to yaw angle.

        Args:
            quaternion: Quaternion message

        Returns:
            Yaw angle in radians
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Using conversion formula
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return yaw


def main(args=None):
    """Main function to start the obstacle avoidance node."""
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
