#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import yaml, time, math, csv, statistics
from nav2_msgs.srv import ClearEntireCostmap

class AutoEvaluator(Node):
    def __init__(self):
        super().__init__('auto_evaluator')

        # --- config読み込み ---
        self.declare_parameter('config_path', '../config/config.yaml')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.routes = config['routes']
        self.timeout = config.get('timeout', 60.0)
        self.trials = config.get('trials_per_route', 100)
        self.success_flag_topic = config.get('success_flag_topic', '/nav2/success_flag')

        # --- Publishers ---
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.reset_pub = self.create_publisher(Pose, '/env/reset_with_pose', 10)

        # --- Subscribers ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_sub = self.create_subscription(String, '/status', self.status_cb, 10)
        self.termination_sub = self.create_subscription(String, '/termination_reason', self.term_cb, 10)
        # === Costmapクリア用クライアント ===
        self.clear_local_cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # ✅ success_flagトピックを購読
        self.success_sub = self.create_subscription(
            Bool,
            self.success_flag_topic,
            self.success_cb,
            10
        )

        # --- TFリスナー ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 状態変数 ---
        self.termination_reason = None
        self.goal_reached = False
        self.results = []

        # 各試行の生データ行を貯める（route/trial/time など）
        self.trial_rows = []

        self.get_logger().info(f"Subscribed to success flag topic: {self.success_flag_topic}")

    # --- Callbacks ---
    def status_cb(self, msg):
        self.current_status = msg.data

    def term_cb(self, msg):
        self.termination_reason = msg.data

    def success_cb(self, msg: Bool):
        """Nav2 success flagを監視"""
        if msg.data:
            self.goal_reached = True
            self.get_logger().info("✅ Goal success flag detected (True)")

    # --- Utility ---
    def yaw_to_quat(self, yaw):
        """yaw[rad] -> quaternion(z,w)"""
        return math.sin(yaw/2), math.cos(yaw/2)

    def publish_reset_pose(self, pose):
        """Isaac Simに初期位置を設定"""
        p = Pose()
        z, w = self.yaw_to_quat(pose['yaw'])
        p.position.x, p.position.y, p.position.z = pose['x'], pose['y'], 0.8
        p.orientation.z, p.orientation.w = z, w
        self.reset_pub.publish(p)
        self.get_logger().info(f"Sent /env/reset_with_pose: {pose}")

    def publish_initial_pose(self, pose):
        """Nav2に初期位置を設定"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        z, w = self.yaw_to_quat(pose['yaw'])
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose['x'], pose['y']
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = z, w
        self.init_pub.publish(msg)
        self.get_logger().info(f"Sent /initial_pose: {pose}")

    def publish_goal(self, goal):
        """Nav2にゴールを送信"""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        z, w = self.yaw_to_quat(goal['yaw'])
        msg.pose.position.x, msg.pose.position.y = goal['x'], goal['y']
        msg.pose.orientation.z, msg.pose.orientation.w = z, w
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Sent /goal_pose: {goal}")

    def get_current_base_link_pose(self):
        """TFから map→base_link の現在位置を取得"""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None

    # --- Main Evaluation ---
    def run_trial(self, route_id, start, goal):
        # --- yawにランダム性を加える ---
        start = start.copy()  # 元の辞書を壊さないためコピー
        yaw_offset = math.radians(random.uniform(-90, 90))  # -90〜90度をラジアンに変換
        start['yaw'] += yaw_offset
        self.get_logger().info(f"Randomized start yaw: {math.degrees(start['yaw']):.1f}°")

        for _ in range(2):  # 初期位置が適用されないときがあるため
            self.publish_reset_pose(start)
            time.sleep(1.0)

        self.publish_initial_pose(start)
        time.sleep(1.0)
        
        self.publish_goal(goal)
        # --- costmapをクリア ---
        self.clear_local_costmap()
        time.sleep(1.0)

        self.goal_reached = False
        self.termination_reason = None
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start_time

            if self.termination_reason == "bad_orientation":
                self.get_logger().warn("Terminated: bad_orientation")
                return {"success": False, "time": None, "error": None, "reason": "bad_orientation"}

            if elapsed > self.timeout:
                self.get_logger().warn(f"Timeout after {elapsed:.1f}s")
                return {"success": False, "time": None, "error": None, "reason": "timeout"}

            if self.goal_reached:
                x, y = self.get_current_base_link_pose()
                if x is not None and y is not None:
                    dx = goal['x'] - x
                    dy = goal['y'] - y
                    error = math.sqrt(dx**2 + dy**2)
                    self.get_logger().info(f"Reached goal: pos=({x:.2f}, {y:.2f}), error={error:.2f} m")
                else:
                    error = None
                    self.get_logger().warn("TF not available for base_link position.")
                elapsed = time.time() - start_time
                return {"success": True, "time": elapsed, "error": error, "reason": "succeeded"}

        return {"success": False, "time": None, "error": None, "reason": "unknown"}

    # def run(self):
    #     for route_idx, route in enumerate(self.routes):
    #         start, goal = route['start'], route['goal']
    #         self.get_logger().info(f"=== Route {route_idx+1}: {start} → {goal} ===")

    #         trial_results = []
    #         for i in range(self.trials):
    #             self.get_logger().info(f"Trial {i+1}/{self.trials}")
    #             res = self.run_trial(route_idx, start, goal)
    #             trial_results.append(res)
    #             time.sleep(3.0)

    #         self.summarize(route_idx, start, goal, trial_results)
    #     self.write_csv()

    def run(self):
        for route_idx, route in enumerate(self.routes):
            start, goal = route['start'], route['goal']
            self.get_logger().info(f"=== Route {route_idx+1}: {start} → {goal} ===")

            trial_results = []
            for i in range(self.trials):
                self.get_logger().info(f"Trial {i+1}/{self.trials}")
                res = self.run_trial(route_idx, start, goal)
                trial_results.append(res)

                # ★ 各試行の行を追加
                self.trial_rows.append({
                    "route": route_idx + 1,
                    "trial": i + 1,
                    "success": res["success"],
                    "time": res["time"],
                    "error": res["error"],
                    "reason": res["reason"],
                })

                time.sleep(3.0)

            self.summarize(route_idx, start, goal, trial_results)

        self.write_csv()

    def summarize(self, route_idx, start, goal, data):
        success = [d for d in data if d["success"]]
        success_rate = len(success) / len(data) * 100
        times = [d["time"] for d in success if d["time"]]
        errors = [d["error"] for d in success if d["error"]]

        avg_time = statistics.mean(times) if times else None
        avg_error = statistics.mean(errors) if errors else None

        avg_time_str = f"{avg_time:.2f}" if avg_time is not None else "N/A"
        avg_error_str = f"{avg_error:.2f}" if avg_error is not None else "N/A"

        self.get_logger().info(
            f"[Route {route_idx+1}] Success={success_rate:.1f}% "
            f"AvgTime={avg_time_str}s AvgError={avg_error_str}m"
        )

        self.results.append({
            "route": route_idx+1,
            "start": start,
            "goal": goal,
            "success_rate": success_rate,
            "avg_time": avg_time,
            "avg_error": avg_error
        })

    def write_csv(self):
        # ① サマリ（既存と同様）
        with open('auto_eval_results.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
            writer.writeheader()
            writer.writerows(self.results)
        self.get_logger().info("Saved results to auto_eval_results.csv")

        # ② 各試行の詳細
        with open('auto_eval_trial_results.csv', 'w', newline='') as f:
            fieldnames = ["route", "trial", "success", "time", "error", "reason"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.trial_rows)
        self.get_logger().info("Saved trial details to auto_eval_trial_results.csv")

    # def write_csv(self):
    #     with open('auto_eval_trial_results.csv', 'w', newline='') as f:
    #         # 各trial単位で保存するように修正
    #         fieldnames = ['route', 'trial', 'success', 'time', 'error', 'reason']
    #         writer = csv.DictWriter(f, fieldnames=fieldnames)
    #         writer.writeheader()

    #         for route_idx, route in enumerate(self.routes):
    #             # 各routeの結果をループ
    #             start, goal = route['start'], route['goal']
    #             for i, res in enumerate(self.results_by_route[route_idx]):
    #                 row = {
    #                     'route': route_idx + 1,
    #                     'trial': i + 1,
    #                     'success': res["success"],
    #                     'time': res["time"],
    #                     'error': res["error"],
    #                     'reason': res["reason"]
    #                 }
    #                 writer.writerow(row)

    #     self.get_logger().info("Saved detailed trial results to auto_eval_trial_results.csv")


    #     # with open('auto_eval_results.csv', 'w', newline='') as f:
    #     #     writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
    #     #     writer.writeheader()
    #     #     writer.writerows(self.results)
    #     # self.get_logger().info("Saved results to auto_eval_results.csv")

    def clear_local_costmap(self):
        """Nav2のlocal costmapをクリア"""
        if not self.clear_local_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ clear_entirely_local_costmap service not available")
            return

        req = ClearEntireCostmap.Request()
        future = self.clear_local_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info("🧹 Local costmap cleared.")
        else:
            self.get_logger().warn("Failed to clear local costmap.")


def main():
    rclpy.init()
    node = AutoEvaluator()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
