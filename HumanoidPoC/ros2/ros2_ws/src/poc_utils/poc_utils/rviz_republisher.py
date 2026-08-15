#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class RvizRepublisher(Node):
    def __init__(self):
        super().__init__('rviz_republisher')

        # --- QoS設定 ---
        qos_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        qos_default = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # --- Publishers ---
        self.pub_map = self.create_publisher(OccupancyGrid, '/rviz2/map', 10)
        self.pub_costmap = self.create_publisher(OccupancyGrid, '/rviz2/global_costmap/costmap', 10)
        self.pub_updates = self.create_publisher(OccupancyGridUpdate, '/rviz2/global_costmap/costmap_updates', 10)

        # --- Subscribers ---
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_transient)
        self.sub_costmap = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, qos_default)
        self.sub_updates = self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates', self.updates_callback, qos_default)

        # --- Timerで周期的にmapを再送 ---
        self.timer_period = 1.0  # 秒
        self.timer = self.create_timer(self.timer_period, self.publish_latest_map)

        self.latest_map = None

        self.get_logger().info("Republishing topics for RViz under /rviz2/*")

    def map_callback(self, msg):
        self.latest_map = msg
        self.get_logger().info("Received /map (stored for periodic republish)")

    def publish_latest_map(self):
        if self.latest_map is not None:
            self.pub_map.publish(self.latest_map)
        # else:
        #     self.get_logger().debug("No map received yet")

    def costmap_callback(self, msg):
        self.pub_costmap.publish(msg)

    def updates_callback(self, msg):
        self.pub_updates.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RvizRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
