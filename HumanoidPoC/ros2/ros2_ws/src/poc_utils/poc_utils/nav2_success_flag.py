#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class Nav2SuccessFlag(Node):
    """
    Nav2のゴールステータスを監視し、
    SUCCEEDED（status=4）になったら /nav2/success_flag に True をpublishするノード
    """

    def __init__(self):
        super().__init__('nav2_success_flag')

        # --- Subscriber ---
        self.sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_cb,
            10
        )


        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # --- Publisher ---
        self.pub = self.create_publisher(Bool, '/nav2/success_flag', qos)

        # --- State tracking ---
        self.last_status = None

        # --- 定義（Nav2 status code mapping）---
        self.status_texts = {
            0: "UNKNOWN",
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED ✅",
            5: "CANCELED ❌",
            6: "ABORTED ⚠️",
        }

        self.get_logger().info("Subscribed to /navigate_to_pose/_action/status")
        self.get_logger().info("Publishing success flag to /nav2/success_flag")

    def status_cb(self, msg: GoalStatusArray):
        """Nav2のGoalStatusArrayを監視し、成功したらTrueをpublish"""
        if not msg.status_list:
            return

        # 最新のstatusを取得
        latest_status = msg.status_list[-1].status

        # 状態が変わったときのみログ出力
        if latest_status != self.last_status:
            self.last_status = latest_status
            status_text = self.status_texts.get(latest_status, f"UNKNOWN({latest_status})")
            self.get_logger().info(f"Nav2 status: {status_text}")

            # 成功時にTrue、それ以外はFalseをpublish
            msg_out = Bool()
            msg_out.data = (latest_status == 4)
            self.pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2SuccessFlag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
