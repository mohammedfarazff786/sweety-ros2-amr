#!/usr/bin/env python3
"""SLAM node for Sweety ROS2 AMR.

This node implements SLAM (Simultaneous Localization and Mapping)
functionality for the Sweety autonomous mobile robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SlamNode(Node):
    """SLAM node for simultaneous localization and mapping.

    This node subscribes to laser scan and odometry data, processes them
    for SLAM algorithms, and publishes the generated map and pose estimates.
    """

    def __init__(self):
        """Initialize the SLAM node."""
        super().__init__('slam_node')

        # QoS profile for sensor data
        sensor_qos = QoSProfile(depth=10)

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            sensor_qos
        )

        # Publications
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'slam_pose',
            10
        )

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Node state
        self.current_pose = Pose()
        self.map_data = OccupancyGrid()

        self.get_logger().info('SLAM node initialized')

    def scan_callback(self, msg: LaserScan) -> None:
        """Process laser scan data.

        Args:
            msg: LaserScan message containing scan data.
        """
        self.get_logger().debug(
            f'Received scan with {len(msg.ranges)} ranges'
        )
        # TODO: Implement scan processing for SLAM

    def odom_callback(self, msg: Odometry) -> None:
        """Process odometry data.

        Args:
            msg: Odometry message containing pose and twist data.
        """
        self.current_pose = msg.pose.pose
        self.get_logger().debug(
            f'Received odometry: x={self.current_pose.position.x:.2f}, '
            f'y={self.current_pose.position.y:.2f}'
        )

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = self.current_pose
        self.pose_publisher.publish(pose_msg)

        # Broadcast transform
        self.broadcast_transform(msg)

    def broadcast_transform(self, odom_msg: Odometry) -> None:
        """Broadcast map to base_link transform.

        Args:
            odom_msg: Odometry message for transform data.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.current_pose.position.x
        transform.transform.translation.y = self.current_pose.position.y
        transform.transform.translation.z = self.current_pose.position.z

        transform.transform.rotation = self.current_pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """Main entry point for SLAM node.

    Args:
        args: Command line arguments.
    """
    rclpy.init(args=args)
    slam_node = SlamNode()

    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
