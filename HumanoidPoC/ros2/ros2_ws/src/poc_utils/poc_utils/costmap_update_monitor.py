import numpy as np
import rclpy
from rclpy.node import Node
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class CostmapUpdateMonitor(Node):
    def __init__(self):
        super().__init__("costmap_update_monitor")

        self.threshold_cost = 80
        self.trigger_count_threshold = 200
        self.map_width = None
        self.map_height = None
        self.global_data = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- costmap本体を購読（最初の1回だけ幅・高さを取得） ---
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.map_cb,
            qos,
        )

        # --- costmap更新トピックを購読 ---
        self.sub_updates = self.create_subscription(
            OccupancyGridUpdate,
            "/global_costmap/costmap_updates",
            self.update_cb,
            qos,
        )

        # --- フラグ出力 ---
        self.flag_pub = self.create_publisher(Bool, "/failsafe/new_obstacle_flag", 10)

        self.get_logger().info("Costmap update monitor started — waiting for /global_costmap/costmap.")

    def map_cb(self, msg: OccupancyGrid):
        """最初の1回だけ地図サイズを取得してキャッシュを初期化"""
        if self.global_data is not None:
            return  # すでに初期化済み

        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.global_data = np.array(msg.data, dtype=np.int16).reshape(self.map_height, self.map_width)
        self.get_logger().info(f"Initialized global costmap buffer: {self.map_width}x{self.map_height}")

    def update_cb(self, msg: OccupancyGridUpdate):
        """costmap_updatesから新しい障害物の出現を検出"""
        if self.global_data is None:
            # costmapの初期化がまだ終わっていない
            return

        x0, y0, w, h = msg.x, msg.y, msg.width, msg.height
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # 範囲チェック（ROSは原点が左下なのでslice順注意）
        if y0 + h > self.map_height or x0 + w > self.map_width:
            self.get_logger().warn(f"Update region ({x0},{y0},{w},{h}) out of bounds!")
            return

        prev_slice = self.global_data[y0:y0+h, x0:x0+w]
        new_obstacles = (prev_slice < self.threshold_cost) & (data >= self.threshold_cost)
        new_count = np.count_nonzero(new_obstacles)

        # 更新を反映
        self.global_data[y0:y0+h, x0:x0+w] = data

        # 判定
        if new_count > self.trigger_count_threshold:
            self.get_logger().warn(f"[FAILSAFE] New obstacle detected: count={new_count}")
            self.flag_pub.publish(Bool(data=True))
        else:
            self.flag_pub.publish(Bool(data=False))


def main():
    rclpy.init()
    node = CostmapUpdateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
