# humanoid-isaac-ros2 Nav2 パラメータ設定ガイド

本書は `humanoid-isaac-ros2`（Unitree G1 / Isaac Sim 5.0 + ROS 2 Humble）の **Nav2 パラメータ**を中心に、**設定ファイルの場所・読み込まれ方・各パラメータの意味と本 PoC での設定値**をまとめたものです。さらに §6 では、ナビ動作に直結する**前段の `/scan` 生成（pointcloud_to_laserscan）と自己位置推定（hdl_localization）**の設定も扱い、地図 → 自己位置 → 障害物認識 → 経路計画/追従までの全要素をカバーします。環境構築・ナビ実行の全体手順は別冊「Isaac Sim ナビゲーション環境 構築・ナビ実行ガイド」（`nav2_environment_setup`）を参照してください。

---

## 1. 設定ファイルの場所

Nav2 のパラメータは **1 つの YAML ファイル**にまとまっており、`nav_vnc` コンテナ内のパスで言うと次の場所にあります。

| 種別 | パス（`nav_vnc` 内 / `/volume` マウント） | ホスト側 |
|---|---|---|
| **Nav2 パラメータ（本書の対象）** | `/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml` | `HumanoidPoC/ros2/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml` |
| Behavior Tree（経路再計画の挙動） | `/volume/nav2_cfg/navigate_long_follow_w_replanning_only_if_goal_is_updated.xml` | 同 `…/nav2_cfg/` |
| 参考: 旧版 / 派生 | `nav2_params_maxV1.0.yaml`、`…_for_uzawa.yaml` | 同 `…/nav2_cfg/` |

`/volume` は `nav_vnc` 起動時にホストの `HumanoidPoC/ros2/volume` をマウントしたものです（第3部参照）。**ホスト側エディタでも編集でき、コンテナを作り直しても残ります。**

### 読み込まれ方

Nav2 は第5部の起動コマンドで、この YAML を `params_file:=` 引数として受け取ります。

```bash
# nav_vnc 内（第5部 §5.3.3）
ros2 launch poc_launch bringup_no_amcl.launch.py \
  use_sim_time:=true \
  map:=/volume/maps/lightwheel_map/lightwheel_map_occgrid.yaml \
  params_file:=/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml
```

- `params_file` を別ファイルにすれば、設定一式を丸ごと差し替えられます。
- **この YAML はビルド対象ではありません**（launch 時に読み込むだけ）。編集後は **Nav2 を再起動するだけ**で反映されます（再ビルド不要）。

> **本構成の前提（重要）**: 自己位置推定は **hdl_localization が `map→odom` を供給**します。Nav2 標準の **amcl は使いません**（`bringup_no_amcl`）。そのため YAML 冒頭の `amcl:` セクションは意図的に無効化されています（`alpha1: a` という不正値で amcl を起動失敗させる小細工。`# amclを落とすことが目的` のコメント）。

---

## 2. パラメータの全体像

YAML には Nav2 を構成する各サーバ／ノードの設定が、ノード単位のセクションで並んでいます。

| セクション | 役割 | 本書 |
|---|---|---|
| `amcl` | （未使用）パーティクルフィルタ自己位置推定。本構成では無効化 | §3.1 |
| `bt_navigator` | ナビ全体の振る舞いを記述する Behavior Tree の実行 | §3.2 |
| `controller_server` | 局所経路追従（速度指令 `/cmd_vel` を生成）。**DWB** プランナ | §3.3 |
| `velocity_smoother` | 速度指令の平滑化・上下限クランプ | §3.4 |
| `behavior_server` | 復帰行動（spin / backup / wait） | §3.5 |
| `planner_server` | 大域経路計画（**NavFn**） | §3.6 |
| `smoother_server` | 経路の平滑化 | §3.7 |
| `waypoint_follower` | 経由点巡回 | §3.8 |
| `global_costmap` | 大域コストマップ（静的地図＋障害物＋膨張） | §3.9 |
| `local_costmap` | 局所コストマップ（ローリングウィンドウ） | §3.10 |
| `map_server` | 占有格子地図の配信（`yaml_filename` は launch から） | §3.11 |

全セクション共通で `use_sim_time: true`（Isaac の `/clock` に同期）です。

---

## 3. 各セクションの詳細

> 表は**本 PoC で特に効く／既定から変更したパラメータ**を中心に記載します。網羅的な定義は Nav2 公式ドキュメントを、実値は上記 YAML を一次情報としてください。YAML 内の `# 変更` `# もともとは…` コメントが本 PoC での変更点です。

### 3.1 amcl（未使用）

本構成では使いません。`alpha1: a`（不正値）で**意図的に起動失敗**させ、amcl が `map→odom` を出さないようにしています（その役割は hdl_localization が担う）。

### 3.2 bt_navigator

ナビ全体の流れ（経路計画 → 追従 → 復帰）を記述した Behavior Tree を実行します。

| パラメータ | 値 | 意味 |
|---|---|---|
| `global_frame` / `robot_base_frame` | `map` / `base_link` | 基準座標系 |
| `odom_topic` | `/odom` | オドメトリ入力 |
| `default_nav_to_pose_bt_xml` | `…/navigate_to_pose_w_replanning_and_recovery.xml`（標準） | **単一ゴール時の BT。毎周期グローバル経路を再計画＋復帰**＝障害物が現れたら**待たずに迂回**する |
| `default_nav_through_poses_bt_xml` | 標準の through_poses BT | 複数ゴール巡回時 |
| `bt_loop_duration` / `default_server_timeout` | 10 / 20 | BT ループ周期・サーバ待機 |

#### 障害物への振る舞いの切り替え（迂回 ⇔ 停止して待つ）

障害物が現れたときに **迂回するか／停止して退くのを待つか** は、`bt_navigator.default_nav_to_pose_bt_xml`（どの Behavior Tree を使うか）で決まります。`/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml` を編集し、**Nav2 を再起動（§5.3.3）するだけ**で切り替わります（ビルド不要）。

| 振る舞い | 設定する BT | 動作 |
|---|---|---|
| **迂回（既定・推奨）** | `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`（標準） | 毎周期グローバル経路を引き直し、障害物を**避けて回り込む**。詰まれば spin/backup で復帰 |
| **停止して待つ** | `/volume/nav2_cfg/navigate_long_follow_w_replanning_only_if_goal_is_updated.xml` | ゴール更新時のみ再計画＝障害物が現れると**停止し、退くのを待つ** |

切り替え手順：

```yaml
bt_navigator:
  ros__parameters:
    # 迂回させたいとき（既定）
    default_nav_to_pose_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    # 停止して待たせたいとき（上を # で無効化し、こちらを有効化）
    # default_nav_to_pose_bt_xml: "/volume/nav2_cfg/navigate_long_follow_w_replanning_only_if_goal_is_updated.xml"
```

1. yaml の `default_nav_to_pose_bt_xml` を**使いたい方の1行だけ有効**にする（もう一方は先頭に `#`）。
2. Nav2 を再起動（第5部 §5.3.3 の `bringup_no_amcl.launch.py …` を再実行）。`Managed nodes are active` で反映。

> 迂回をスムーズにするには `local_costmap` の `clearing: true`（§3.10。退いた障害物をクリア）と、`controller_server` の `BaseObstacle.scale`（§3.3）・costmap の `inflation_radius`（§3.9/§3.10）が効きます。

### 3.3 controller_server（局所追従・DWB）

目標経路に追従する速度指令を生成する中核。プラグインは **DWB（`dwb_core::DWBLocalPlanner`）**。

**サーバ全体**

| パラメータ | 値 | 意味 |
|---|---|---|
| `controller_frequency` | 20.0 Hz | 制御ループ周波数 |
| `failure_tolerance` | 0.3 | 失敗許容 |
| `goal_checker` | `xy_goal_tolerance: 0.4` / `yaw_goal_tolerance: 0.5` | **ゴール到達判定**（位置 0.4 m・向き 0.5 rad 以内で到達とみなす） |
| `progress_checker` | `required_movement_radius: 0.5` / `movement_time_allowance: 10.0` | 10 秒で 0.5 m 進めないと「進捗なし」 |

**DWB（`FollowPath`）速度・加速度**

| パラメータ | 値 | 意味 |
|---|---|---|
| `min_vel_x` / `max_vel_x` | 0.0 / **2.0** | 前後方向速度（**前進上限を 2.0 に拡大＝変更点**） |
| `min_vel_y` / `max_vel_y` | -2.0 / 2.0 | 横方向速度（G1 は横移動可） |
| `max_vel_theta` | 2.0 | 旋回速度上限 |
| `max_speed_xy` | 1.0 | 合成並進速度上限 |
| `acc_lim_x/y/theta` | 0.5 / 0.5 / 1.0 | 加速度上限 |
| `decel_lim_x/y/theta` | -0.5 / -0.5 / -1.0 | 減速度上限 |
| `vx/vy/vtheta_samples` | 20 / 20 / 20 | 速度サンプリング数（候補軌道の細かさ） |
| `sim_time` | 1.7 s | 候補軌道の予測時間 |

**DWB 評価（critics）**: `["RotateToGoal","Oscillation","BaseObstacle","GoalAlign","PathAlign","PathDist","GoalDist"]`。各 critic の重み:

| critic | scale | 効果 |
|---|---|---|
| `BaseObstacle` | 40.0 | 障害物回避（最重視） |
| `PathAlign` / `PathDist` | 32.0 / 32.0 | 計画経路への整列・追従 |
| `GoalAlign` / `GoalDist` | 24.0 / 24.0 | ゴール方向への整列・接近 |
| `RotateToGoal` | 10.0 | ゴール手前での向き合わせ |

> 速度を上げたい/下げたいときは主に `max_vel_x` `max_vel_theta` と `velocity_smoother`（§3.4）を、回避の強さは `BaseObstacle.scale` と costmap の `inflation_radius`（§3.9/§3.10）を調整します。

### 3.4 velocity_smoother

`controller_server` の出力を平滑化し、最終的な上下限でクランプします。

| パラメータ | 値 | 意味 |
|---|---|---|
| `max_velocity` | [0.5, 0.5, 1.0] | [x, y, θ] の上限 |
| `min_velocity` | [-0.5, -0.5, -1.0] | 下限（**x を 0.0→-0.5 にし後退を許可＝変更点**） |
| `max_accel` / `max_decel` | [0.5,0.5,1.0] / [-0.5,-0.5,-1.0] | 加減速上限 |
| `smoothing_frequency` | 20.0 Hz | 平滑化周波数 |
| `feedback` | `OPEN_LOOP` | 指令ベース（odom フィードバックを使わない） |

> **実効的な最高速はここで決まります**（DWB の `max_vel_x:2.0` でも、ここの `max_velocity` x=0.5 でクランプ）。きびきび/ゆっくりは主にここを調整。

### 3.5 behavior_server（復帰行動）

詰まったときの復帰行動。プラグイン: `spin`（その場回転）/ `backup`（後退）/ `wait`（待機）。`global_frame: odom`。

### 3.6 planner_server（大域計画・NavFn）

スタート→ゴールの大域経路を計算。プラグインは **NavFn（`nav2_navfn_planner/NavfnPlanner`）**。

| パラメータ | 値 | 意味 |
|---|---|---|
| `tolerance` | 0.25 m | ゴール到達不能時にこの距離まで近い点で代替 |
| `use_astar` | false | Dijkstra（false）/ A*（true） |
| `allow_unknown` | true | 未知セルの通過を許可 |

### 3.7 smoother_server

計算した経路を滑らかにする（`SimpleSmoother`、`max_its: 1000`、`do_refinement: True`）。

### 3.8 waypoint_follower

経由点を順に巡回。`wait_at_waypoint` で各点に到達後 `waypoint_pause_duration: 5000` ms 待機。

### 3.9 global_costmap（大域コストマップ）

地図全体のコストマップ。`map` フレーム、`resolution: 0.05`、`update/publish: 1.0 Hz`、`track_unknown_space: true`。レイヤ: **static + obstacle + inflation**。

| レイヤ | 主パラメータ | 意味 |
|---|---|---|
| `static_layer` | `map_subscribe_transient_local: True` | 占有格子地図（map_server）を取り込む |
| `obstacle_layer` | source=`/scan`、`obstacle_range: 6.0`、`raytrace_range: 8.0`、`min/max_obstacle_height: 0.0/1.5` | **LiDAR 由来 `/scan` で障害物をマーキング/クリア**。高さ 0〜1.5 m を障害物として扱う |
| `inflation_layer` | `inflation_radius: 0.2`、`cost_scaling_factor: 4.0` | 障害物の膨張（安全マージン） |

### 3.10 local_costmap（局所コストマップ）

ロボット周囲のみのローリングウィンドウ。`odom` フレーム、`rolling_window: true`、`width/height: 3`（3×3 m）、`update/publish: 5.0 Hz`。レイヤ: **obstacle + inflation**。

| レイヤ | 主パラメータ | 意味 |
|---|---|---|
| `obstacle_layer` | source=`/scan`、`clearing: true` | 退いた障害物をクリアし最新状態を反映（迂回を素直に効かせる） |
| `inflation_layer` | `inflation_radius: 0.5`、`cost_scaling_factor: 4.0` | 局所は膨張を厚め（0.5 m）にして安全に回避 |

> **障害物を「太く/細く」感じる（避けすぎ/擦る）ときは `inflation_radius`** を、`/scan` の届く範囲は `obstacle_range`/`raytrace_range` を調整します。

### 3.11 map_server

占有格子地図を配信。`yaml_filename` は空（**launch の `map:=` で渡す**）。

---

## 4. パラメータの確認・変更・反映

### 確認

```bash
# nav_vnc 内：ファイルを直接見る
cat /volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml
# 起動中ならライブ確認（例: controller_server）
ros2 param list /controller_server
ros2 param get /controller_server FollowPath.max_vel_x
```

### 変更

`/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml`（ホスト側でも編集可）を編集します。**この YAML はビルド不要**で、Nav2 を再起動すれば反映されます。設定一式を試したい場合は別名で保存し、`params_file:=…` で渡してください。

### 反映

Nav2 を一度停止し、第5部 §5.3.3 の `ros2 launch poc_launch bringup_no_amcl.launch.py …` を再実行します（`Managed nodes are active` が出れば反映完了）。

---

## 5. ビルド方法（ROS 2 ワークスペース）

Nav2 本体（`navigation2`）や本 PoC の起動パッケージ（`poc_launch` / `poc_utils`）、`pointcloud_to_laserscan`、`pcd2pgm` などは **`nav_vnc` コンテナ内の ROS 2 ワークスペース `/ros2_ws`** でビルドします。

> **重要**: 上記の **Nav2 パラメータ YAML（`/volume/nav2_cfg/…`）と BT XML はビルド対象ではありません**（launch 時に読み込むだけ）。再ビルドが要るのは **ソースコード（`/ros2_ws/src`）を変更したとき**だけです。

### 5.1 一括ビルド（推奨・同梱スクリプト）

新規 `nav_vnc` では、同梱スクリプトが clone＋依存導入＋`colcon build` をまとめて実行します（環境構築ガイド第3部 §3.2 と同じ）。

```bash
docker exec -it nav_vnc bash -lc '/volume/setup_scripts/setup.sh'
```

- `rmw_cyclonedds_cpp`・PCL などの apt 依存も導入されます（**コンテナごとに必須**）。
- `hdl_global_localization` は `nlohmann/json` の競合でビルドが止まることがありますが、**自己位置推定は事前ビルド済みの hdl Docker イメージで動かす**ため、`poc_launch` / `poc_utils` が建っていれば先へ進めます。

### 5.2 手動ビルド（ソース変更後）

`/ros2_ws/src` 配下を編集した後に作り直す場合:

```bash
# nav_vnc 内
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash          # ビルド成果を反映（各ターミナルで）
```

- 特定パッケージだけ: `colcon build --symlink-install --packages-select poc_launch`
- `--symlink-install` を付けると、launch ファイルや設定の軽微な変更が再ビルドなしで反映されやすくなります。

### 5.3 ビルド確認

```bash
docker exec -it nav_vnc bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 pkg list | grep -E "poc_launch|poc_utils|pcd2pgm|pointcloud_to_laserscan"'
```

`poc_launch` / `poc_utils` が表示されれば、第5部のナビ起動に必要なパッケージは揃っています。

---

## 6. ナビゲーション関連のその他の設定

ナビゲーションは Nav2 単体ではなく、**前段の `/scan` 生成**と**自己位置推定**にも依存します。これらは別ファイルで、Nav2 の YAML には含まれません。

### 6.1 pointcloud_to_laserscan（/lidar/points → /scan）

3D 点群を 2D の `/scan` に変換するノード。**Nav2 のコストマップ `obstacle_layer` はこの `/scan` を入力にする**ため、ここで「何を障害物として拾うか」が決まります。

- 場所: `/volume/pc2_to_scan_cfg/pc2_to_scan.yaml`（ホスト: `HumanoidPoC/ros2/volume/pc2_to_scan_cfg/pc2_to_scan.yaml`）
- 起動: 第5部 §5.3.2。**変更後はこのノードだけ再起動**で反映。

| パラメータ | 値 | 意味 |
|---|---|---|
| `target_frame` | `base_link` | `/scan` の基準座標 |
| **`min_height` / `max_height`** | **-0.54 / 0.5** | **点群のうちこの高さ帯だけを `/scan` に落とす＝障害物として見る高さ範囲。** 低い机・段差を拾うか、床/天井を除外するかをここで決める（costmap の `min/max_obstacle_height` §3.9/§3.10 と二段で効く） |
| `angle_min` / `angle_max` | -3.14 / 3.14 | 視野角（全方位 360°） |
| `angle_increment` | 0.0087 | 角度分解能（約 0.5°） |
| `range_min` / `range_max` | 0.1 / 10.0 | 検出距離（10 m 先まで） |
| `use_inf` | true | 範囲外を `inf` で表現 |

> 机など低い障害物が避けられない場合は `min_height`/`max_height` を見直します（机天面が帯に入っているか）。遠くの障害物に早く反応させたいなら `range_max` と costmap の `obstacle_range`/`raytrace_range` を合わせます。

### 6.2 hdl_localization（自己位置推定・map→odom）

点群地図に対する自己位置推定。**Nav2 が必要とする `map→odom` の TF を供給**します（amcl の代替）。

- 場所: `HumanoidPoC/ros2/docker/param/localization.yaml`（`docker-compose.hdl_localization.yml` が hdl コンテナへマウント）
- 起動: 第5部 §5.3.1。**変更後は hdl の `docker compose` を再起動**で反映。

| パラメータ | 値 | 意味 |
|---|---|---|
| `map_frame` / `odom_frame` / `base_frame` | map / odom / base_link | フレーム構成 |
| `enable_map_odom_tf` | true | **`map→odom` の TF を出す**（これが Nav2 の自己位置） |
| `reg_method` | `NDT_OMP` | スキャンマッチング手法 |
| `ndt_resolution` | 1.0 | NDT ボクセル解像度（小さいほど精密・重い） |
| `downsample_resolution` | 0.1 | 入力点群の間引き（小さいほど精密・重い） |
| `ndt_neighbor_search_method` / `_radius` | DIRECT7 / 2.0 | 近傍探索 |
| `use_imu` | true | IMU を融合 |
| `enable_robot_odometry_prediction` | false | 車輪 odom 予測を使わない（脚ロボット向け） |
| **`specify_init_pose` / `init_pos_*` / `init_ori_*`** | true / (0,0,0) / 単位姿勢 | **初期位置を地図原点に固定。ロボットの開始位置が地図原点と一致している前提。** ズレる場合は RViz の「2D Pose Estimate」で与えるか `init_pos_*` を調整 |

> 自己位置が合わない／ずれる場合は、まず **開始位置と地図原点の一致**（`init_pos_*` または 2D Pose Estimate）を確認し、精度/負荷は `ndt_resolution`・`downsample_resolution` で調整します。

### 6.3 ナビゲーション設定ファイルの早見表

| 対象 | ファイル | 変更後に再起動するもの | 解説 |
|---|---|---|---|
| Nav2 本体 | `/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml` | Nav2 launch（§5.3.3） | §3 |
| Behavior Tree | `/volume/nav2_cfg/navigate_long_follow_…xml` | Nav2 launch | §3.2 |
| /scan 生成 | `/volume/pc2_to_scan_cfg/pc2_to_scan.yaml` | laserscan ノード（§5.3.2） | §6.1 |
| 自己位置推定 | `docker/param/localization.yaml` | hdl `docker compose`（§5.3.1） | §6.2 |
| 占有格子の生成 | pcd2pgm パラメータ | （地図作成時のみ） | 本体 §4.4.1 |
| 占有格子（地図そのもの） | `…/lightwheel_map_occgrid.yaml/.pgm` | Nav2 launch（`map:=`） | 本体 §4 |

これらを合わせると、**地図 → 自己位置推定（hdl）→ 障害物認識（laserscan→costmap）→ 経路計画/追従（Nav2）** というナビゲーションの全要素の設定がカバーされます。

---

© 2026 トロン株式会社 (TRON K.K.) All Rights Reserved.
