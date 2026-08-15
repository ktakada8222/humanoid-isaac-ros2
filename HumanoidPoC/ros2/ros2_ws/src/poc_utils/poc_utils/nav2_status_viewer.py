#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

class Nav2StatusViewer(Node):
    """Nav2のゴールステータスを監視して表示するノード"""

    def __init__(self):
        super().__init__('nav2_status_viewer')
        self.sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_cb,
            10
        )
        self.last_status = None
        self.status_texts = {
            0: "UNKNOWN",
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED ✅",
            5: "CANCELED ❌",
            6: "ABORTED ⚠️"
        }
        self.get_logger().info("Subscribed to /navigate_to_pose/_action/status")

    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        last = msg.status_list[-1]
        status_code = last.status
        if status_code != self.last_status:
            self.last_status = status_code
            status_text = self.status_texts.get(status_code, f"UNKNOWN({status_code})")
            self.get_logger().info(f"Nav2 status: {status_text}")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2StatusViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
