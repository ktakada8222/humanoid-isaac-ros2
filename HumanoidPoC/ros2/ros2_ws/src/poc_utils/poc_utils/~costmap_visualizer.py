#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapUpdate

class RvizRepublisher(Node):
    def __init__(self):
        super().__init__('rviz_republisher')

        # --- Publishers ---
        self.pub_map = self.create_publisher(OccupancyGrid, '/rviz2/map', 10)
        self.pub_costmap = self.create_publisher(OccupancyGrid, '/rviz2/global_costmap/costmap', 10)
        self.pub_updates = self.create_publisher(CostmapUpdate, '/rviz2/global_costmap/costmap_updates', 10)

        # --- Subscribers ---
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_costmap = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.sub_updates = self.create_subscription(CostmapUpdate, '/global_costmap/costmap_updates', self.updates_callback, 10)

        self.get_logger().info("Republishing topics for RViz under /rviz2/*")

    def map_callback(self, msg):
        self.pub_map.publish(msg)

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
