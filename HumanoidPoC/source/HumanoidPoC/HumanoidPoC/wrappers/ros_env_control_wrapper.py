# ---------- 標準・依存ライブラリ ----------
import gym                          # Gym環境ラッパーの基底クラスとして使用
import rclpy                        # ROS2 Pythonクライアントライブラリ
from rclpy.node import Node         # ROS2ノードの基底クラス
from rclpy.qos import QoSProfile    # QoS設定を行うためのクラス
from std_srvs.srv import Trigger    # 環境リセット用の単純なサービス
from geometry_msgs.msg import Pose, PoseStamped  # 位置姿勢データ型
from std_msgs.msg import String     # テキストメッセージ型（キューブ削除用）
import numpy as np
from pxr import UsdGeom, Gf         # Isaac Sim内部でUSDプリム操作に使用
from HumanoidPoC.utils.reset_pose_utils import reset_root_state_to_pose
import isaaclab.sim as sim_utils
from scipy.spatial.transform import Rotation as R

# =========================================================
# ROSノードクラス（環境操作を担当）
# =========================================================
class _RosEnvControlNode(Node):
    """Isaac Sim内の環境を制御するためのROSノード"""
    def __init__(self, sim, env, use_sim_time=False):
        # ROSノード名を定義して初期化
        super().__init__('isaac_lab_env_control')
        self.sim = sim     # IsaacSimのsimulationハンドル
        self.env = env     # Gym環境
        self.use_sim_time = use_sim_time  # IsaacSimのシミュレーション時間を使うかどうか

        # Isaac Simの /clock と同期する場合はパラメータを設定
        if use_sim_time:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL,
                    True
                )
            ])

        # QoS設定（通信の信頼性などを定義）
        qos = QoSProfile(depth=5)

        # === サービス（環境リセット） ===
        # /env/reset → 通常リセット（std_srvs/Trigger型）
        self.srv_reset = self.create_service(Trigger, '/env/reset', self.cb_reset)
        # /env/reset_with_pose → 初期位置を指定してリセット（geometry_msgs/Pose型）
        self.sub_reset_pose = self.create_subscription(
            Pose, '/env/reset_with_pose', self.cb_reset_pose, qos
        )

        # === パブリッシャ（ロボットの位置配信） ===
        # ロボットのworld座標をPoseStampedで配信
        self.pub_pose = self.create_publisher(PoseStamped, '/robot/world_pose', qos)

        # Termination Reason Publisher
        self.pub_termination_reason = self.create_publisher(String, "/termination_reason", qos)

        # === サブスクライバ（キューブ制御） ===
        # /spawn_cube: Poseを受け取ってキューブを生成
        self.sub_spawn_cube = self.create_subscription(Pose, '/spawn_cube', self.cb_spawn_cube, qos)
        # /delete_cube: Stringを受け取ってキューブを削除
        self.sub_delete_cube = self.create_subscription(String, '/delete_cube', self.cb_delete_cube, qos)

        # キューブ管理のための内部変数
        self.cube_counter = 0
        self.spawned_cubes = {}

    # -------------------------------------------------
    # 1️⃣ 通常リセット用のコールバック関数
    # -------------------------------------------------
    def cb_reset(self, request, response):
        """/env/reset サービス呼び出し時に実行される"""
        self.get_logger().info("Resetting environment.")
        self.env.reset()  # Gym環境をリセット
        response.success = True
        response.message = "Environment reset"
        return response

    # -------------------------------------------------
    # 2️⃣ 初期位置指定付きリセット
    # -------------------------------------------------
    # def cb_reset_pose(self, msg):
    #     self.get_logger().info(f"Reset with pose: ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})")

    #     self.env.reset()
    #     from HumanoidPoC.utils.reset_pose_utils import reset_root_state_to_pose
    #     reset_root_state_to_pose(
    #         self.env,
    #         pose_xyz=(msg.position.x, msg.position.y, max(0.5, msg.position.z)),
    #         quat_xyzw=(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    #     )
    def cb_reset_pose(self, msg):
        self.get_logger().info(
            f"Reset with quaternion pose: "
            f"({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}) "
            f"quat=({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f})"
        )

        # IsaacLab環境リセット
        self.env.reset()

        # クォータニオンをそのまま使用
        reset_root_state_to_pose(
            self.env,
            pose_xyz=(msg.position.x, msg.position.y, max(0.5, msg.position.z)),
            quat_xyzw=(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ),
        )

        self.get_logger().info("Applied pose using received quaternion.")

    # -------------------------------------------------
    # 3️⃣ キューブ生成
    # -------------------------------------------------
    def cb_spawn_cube(self, msg):
        """/spawn_cube を受け取ったら Isaac Lab の Spawner でキューブ生成"""
        # 一意な prim path
        cube_name = f"/World/cube_{self.cube_counter}"
        self.cube_counter += 1

        # CuboidCfg を利用して立方体を spawn
        from isaaclab.sim.spawners.shapes import CuboidCfg
        cfg = CuboidCfg(
            size=(1.0, 1.0, 1.0),  # 幅・高さ・奥行き
            rigid_props=None,  # 物理特性を持たせたい場合
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.6, 0.8))
        )
        # cfg.func = 実際の spawn 関数への参照
        cfg.func(cube_name, cfg, translation=(msg.position.x, msg.position.y, msg.position.z))

        self.spawned_cubes[cube_name] = cube_name
        self.get_logger().info(f"Spawned cube via spawner: {cube_name}")

    def cb_delete_cube(self, msg):
        name = msg.data
        if name in self.spawned_cubes:
            import omni.kit.commands
            omni.kit.commands.execute("DeletePrims", paths=[name])
            del self.spawned_cubes[name]
            self.get_logger().info(f"Deleted cube (spawner): {name}")
        else:
            self.get_logger().warn(f"Cube {name} not found")

    # -------------------------------------------------
    # 5️⃣ ロボット位置をROSトピックで配信
    # -------------------------------------------------
    def publish_robot_pose(self, pose):
        """
        IsaacLabのsceneから取得したロボット姿勢を
        /robot/world_pose として配信
        """
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()

        # pose = [x, y, z, qw, qx, qy, qz] の想定
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        msg.pose.orientation.w = pose[3]
        msg.pose.orientation.x = pose[4]
        msg.pose.orientation.y = pose[5]
        msg.pose.orientation.z = pose[6]

        # publish!
        self.pub_pose.publish(msg)


# =========================================================
# Gym環境のラッパークラス
# =========================================================
class IsaacRosEnvControlWrapper(gym.Wrapper):
    """
    環境リセットや物体生成・削除をROS2トピック/サービスで制御するラッパ。
    Gym環境を包み込み、毎ステップでROSをspinしてコールバックを処理。
    """

    def __init__(self, env, sim, use_sim_time=True):
        super().__init__(env)
        self.sim = sim

        # --- ROS初期化 ---
        if not rclpy.ok():                 # まだrclpy.initしてなければ
            rclpy.init(args=None)
            self._shutdown_on_close = True  # 終了時に自動シャットダウン
        else:
            self._shutdown_on_close = False

        # --- ROSノード生成 ---
        self.node = _RosEnvControlNode(sim, env, use_sim_time)

    # -------------------------------------------------
    # Gym step() 拡張
    # -------------------------------------------------
    def step(self, action):
        """毎ステップごとにROSコールバックをspinし、位置を配信"""
        obs, rew, done, trunc, info = self.env.step(action)

        # ROSコールバックを処理（非ブロッキング）
        rclpy.spin_once(self.node, timeout_sec=0.0)

        def _as_bool(x):
            try:
                return bool(x.any()) if hasattr(x, "any") else bool(x)
            except Exception:
                return False

        terminated = _as_bool(done)
        truncated = _as_bool(trunc)

        if terminated or truncated:
            print("Episode ended.")

            log = info.get("log", {})
            reasons = []

            # IsaacLab のログ情報から termination 条件を抽出
            for key, value in log.items():
                if key.startswith("Episode_Termination/") and float(value) > 0.5:
                    reason_name = key.split("Episode_Termination/")[-1]
                    reasons.append(reason_name)

            # truncated (時間切れ) も補助的に追加
            if truncated and "time_out" not in reasons and "timeout" not in reasons:
                reasons.append("time_out")

            # ROS トピックとして出力
            if reasons:
                msg = String()
                msg.data = ", ".join(reasons)
                self.node.pub_termination_reason.publish(msg)
                self.node.get_logger().info(f"Published termination reason: {msg.data}")
            else:
                self.node.get_logger().info("Episode ended but no specific reason found.")

        return obs, rew, done, trunc, info

    # -------------------------------------------------
    # 終了時処理
    # -------------------------------------------------
    def close(self):
        """環境終了時にROSノードを破棄"""
        try:
            super().close()
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            if self._shutdown_on_close and rclpy.ok():
                rclpy.shutdown()
