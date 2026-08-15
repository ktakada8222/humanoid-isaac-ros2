import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_msgs.srv import ManageLifecycleNodes
import time

class LidarFailsafeNode(Node):
    def __init__(self):
        super().__init__("lidar_failsafe_node")

        # ===== パラメータ =====
        self.declare_parameter("timeout_sec", 3.0)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        # ===== 状態管理 =====
        self.last_msg_time = self.get_clock().now()
        self.nav_shutdown_sent = False

        # ===== 通信設定 =====
        self.sub_lidar = self.create_subscription(PointCloud2, "/lidar/points", self.lidar_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_stop_flag = self.create_publisher(Bool, "/failsafe/stop_nav", 10)

        # Nav2ライフサイクルマネージャへのクライアント
        self.lifecycle_client = self.create_client(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")

        # ===== タイマー =====
        self.timer = self.create_timer(0.5, self.check_timeout)

        self.get_logger().info(f"[Failsafe] Node started. Timeout = {self.timeout_sec:.1f} sec")

    # --------------------------------------------------
    # LiDAR受信
    # --------------------------------------------------
    def lidar_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        if self.nav_shutdown_sent:
            self.nav_shutdown_sent = False
            self.get_logger().info("[Failsafe] LIDAR signal recovered.")

    # --------------------------------------------------
    # タイムアウト監視
    # --------------------------------------------------
    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            self.get_logger().warn(f"[Failsafe] No LIDAR data for {elapsed:.1f}s → STOP triggered")
            if not self.nav_shutdown_sent:
                self.stop_nav2_and_robot()
                self.nav_shutdown_sent = True
            stop_flag = Bool(data=True)
        else:
            stop_flag = Bool(data=False)
        self.pub_stop_flag.publish(stop_flag)

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

        # Nav2が止まるまでの少しの間、確実に0速度を送る
        self.get_logger().info("[Failsafe] Publishing zero /cmd_vel for 2 seconds...")
        twist = Twist()
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.05)  # 20Hzで送信

        self.get_logger().info("[Failsafe] Robot should now be fully stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarFailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
