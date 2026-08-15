#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ManageLifecycleNodes
import time


class NewObstacleFailsafeNode(Node):
    def __init__(self):
        super().__init__("new_obstacle_failsafe_node")

        # ===== パラメータ =====
        self.declare_parameter("stop_duration_sec", 2.0)
        self.stop_duration_sec = float(self.get_parameter("stop_duration_sec").value)

        # ===== 状態管理 =====
        self.nav_shutdown_sent = False

        # ===== 通信設定 =====
        self.sub_flag = self.create_subscription(
            Bool, "/failsafe/new_obstacle_flag", self.flag_callback, 10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        # Nav2ライフサイクルマネージャへのクライアント
        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )

        self.get_logger().info("[Failsafe] NewObstacleFailsafeNode started.")

    # --------------------------------------------------
    # フラグ受信
    # --------------------------------------------------
    def flag_callback(self, msg: Bool):
        """障害物増加フラグを受信"""
        if msg.data:
            if not self.nav_shutdown_sent:
                self.get_logger().warn("[Failsafe] New obstacle detected → shutting down Nav2.")
                self.stop_nav2_and_robot()
                self.nav_shutdown_sent = True
            else:
                self.get_logger().debug("[Failsafe] Shutdown already sent; ignoring repeat True.")
        else:
            # フラグがFalseに戻ったら、再度検知を受け入れ可能にする
            if self.nav_shutdown_sent:
                self.get_logger().info("[Failsafe] Flag reset. Ready for next trigger.")
            self.nav_shutdown_sent = False

    # --------------------------------------------------
    # Nav2停止＋ロボット停止
    # --------------------------------------------------
    def stop_nav2_and_robot(self):
        """Nav2をシャットダウンしてロボットを停止"""
        # Nav2停止
        if not self.lifecycle_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[Failsafe] lifecycle_manager_navigation service not available.")
        else:
            req = ManageLifecycleNodes.Request()
            req.command = 3  # SHUTDOWN
            self.lifecycle_client.call_async(req)
            self.get_logger().info("[Failsafe] Sent Nav2 shutdown command.")

        # Nav2が止まるまでの間、安全のため停止速度を送る
        self.get_logger().info("[Failsafe] Publishing zero /cmd_vel for {:.1f} seconds...".format(self.stop_duration_sec))
        twist = Twist()
        start_time = time.time()
        while time.time() - start_time < self.stop_duration_sec:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.05)  # 20Hz
        self.get_logger().info("[Failsafe] Robot should now be fully stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = NewObstacleFailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
