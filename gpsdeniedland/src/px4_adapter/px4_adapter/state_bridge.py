"""
Node:
    State Bridge

Purpose:
    Converts MAVROS Odometry into the project's VehicleState message.

Input:
    /mavros/local_position/odom

Output:
    /vehicle/state

Future:
    Subscribe to /mavros/state to publish
    armed status and flight mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from gps_navigation_interfaces.msg import VehicleState

from tf_transformations import euler_from_quaternion


class StateBridge(Node):

    def __init__(self):
        super().__init__("state_bridge")

        # Reuse the same message object
        self.vehicle_state = VehicleState()

        # Flag to detect first MAVROS message
        self.first_message = True

        # MAVROS publishes local_position/odom with BEST_EFFORT reliability.
        # We MUST match it or no messages are received (QoS incompatibility),
        # which would leave guidance_controller with no vehicle state.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher
        self.vehicle_state_pub = self.create_publisher(
            VehicleState,
            "/vehicle/state",
            5
        )

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_callback,
            sensor_qos
        )

        self.get_logger().info("===================================")
        self.get_logger().info(" PX4 State Bridge Started")
        self.get_logger().info(" Waiting for MAVROS Odometry...")
        self.get_logger().info("===================================")

    # ----------------------------------------------------------
    # Callback: Called whenever MAVROS publishes Odometry
    # ----------------------------------------------------------
    def odom_callback(self, msg):

        if self.first_message:
            self.get_logger().info(" MAVROS Odometry Connected.")
            self.first_message = False

        # Preserve PX4 timestamp
        self.vehicle_state.header.stamp = msg.header.stamp

        # Use project-wide frame
        self.vehicle_state.header.frame_id = "map"

        # Position
        self.vehicle_state.x = msg.pose.pose.position.x
        self.vehicle_state.y = msg.pose.pose.position.y
        self.vehicle_state.z = msg.pose.pose.position.z

        # Linear Velocity
        self.vehicle_state.vx = msg.twist.twist.linear.x
        self.vehicle_state.vy = msg.twist.twist.linear.y
        self.vehicle_state.vz = msg.twist.twist.linear.z

        # Quaternion -> Yaw
        q = msg.pose.pose.orientation

        quaternion = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        _, _, yaw = euler_from_quaternion(quaternion)

        # Yaw in radians
        self.vehicle_state.yaw = yaw

        # Publish
        self.vehicle_state_pub.publish(self.vehicle_state)

        # TODO:
        # Subscribe to /mavros/state
        # Publish armed status and flight mode


def main(args=None):

    rclpy.init(args=args)

    node = StateBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
