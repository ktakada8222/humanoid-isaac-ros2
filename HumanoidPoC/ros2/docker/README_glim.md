# GLIM (ROS 2) 実行方法

この手順では、`docker/` 配下の compose で、
ホストのLiDARトピックを購読しつつ（Unitree L2: `/unilidar/cloud`, `/unilidar/imu`）、
GLIMノードとRViz2をコンテナ内で同時起動します。ビルドは行わず、`koide3/glim_ros2` の公式イメージを利用します。

## 構成

- コンテナ: `glim-ros2`（`koide3/glim_ros2:humble` または `:humble_cuda12.2` を使用）
- ネットワーク: `network_mode: host`（ホストROS 2のトピックに直接接続）
- 可視化: X11共有でRViz2を起動
- マウント:
  - `slam/glim/config` → `/glim/config`
  - `slam/glim/maps` → `/glim/maps` および `/tmp/dump`（出力保存）

## 事前準備

1. X11の接続許可（Xorgの場合）
   ```bash
   xhost +local:
   ```
   - Waylandの場合はXWayland経由の許可が必要です（ディストリごとの設定に従ってください）。

2. ROS_DOMAIN_ID を合わせる（任意）
   ```bash
   export ROS_DOMAIN_ID=0
   ```
   - ホスト側とコンテナ側の `ROS_DOMAIN_ID` が一致している必要があります。

3. コンフィグの準備（最初に1回）
   - GLIM公式の設定一式をホストに用意してマウントします。
     ```bash
     # GLIMリポジトリのconfigを取得して配置
     git clone https://github.com/koide3/glim /tmp/glim
     rsync -a /tmp/glim/config/ ../slam/glim/config/
     ```
   - Unitree L2 LiDARに合わせて以下を編集（`../slam/glim/config` 内）
     - `config/config.json`:
       - GPUなしの場合: `config_odometry` / `config_sub_mapping` / `config_global_mapping` を `*_cpu.json` に変更
     - `config/config_ros.json`:
       - `"imu_topic": "/unilidar/imu"`
       - `"points_topic": "/unilidar/cloud"`

## ビルドと起動

```bash
cd docker
# CPU版（デフォルト）
docker compose up glim

# GPU版（NVIDIA）を使いたい場合は、環境変数でイメージを切り替え
GLIM_IMAGE=koide3/glim_ros2:humble_cuda12.2 docker compose up glim
```

- 起動すると、GLIMノードが立ち上がると同時にRViz2が表示されます。
  - GLIMノードは `/velodyne/points`（点群）と `/velodyne/points_imu`（IMU）へリマップして起動します。
  - 本composeではGLIM側で `/unilidar/cloud` と `/unilidar/imu` にリマップしています。
  - RVizはデフォルト設定で起動します（必要に応じて任意の`.rviz`を指定してください）。

## 停止

```bash
cd docker
docker compose down
```

## よくある注意点

- LiDARトピックが見つからない:
  - ホストで `/velodyne/points` と `/velodyne/points_imu` が配信されているか確認。
  - `ROS_DOMAIN_ID` が一致しているか確認。
  - 同一マシンでROS 2のミドルウェア（RMW）が異なる場合、`RMW_IMPLEMENTATION` を合わせる。

- RViz2が表示されない/真っ黒:
  - `xhost +local:` を実行したか確認。
  - NVIDIA GPU環境では、ホストに `nvidia-container-toolkit` が導入済みか、
    あるいはソフトウェアレンダリングで一旦試す（現在のcomposeはX11共有のみ）。

- オフラインビューア（点群MAP化の確認）:
  - SLAM実行後の出力は `../slam/glim/maps`（コンテナ内では `/tmp/dump`）に保存されます。
  - オフラインビューア起動（compose経由）:
    ```bash
    docker compose run --rm glim bash -lc "source /opt/ros/humble/setup.bash && ros2 run glim_ros offline_viewer"
    ```
  - GLIMのGUIで File -> Read から `/tmp/dump` を指定し、Save Map で `.ply` を保存するとホストの `../slam/glim/maps` に出力されます。

- ビルドがC++エラーで失敗する:
  - `GLIM_REF` を別のタグ/ブランチに切り替えて再ビルド（例: `GLIM_REF=main`）。
  - もしくは `GLIM_COMMIT` に安定コミットSHAを指定して固定。
  - タグ/ブランチ一覧の確認例: `git ls-remote --tags --heads https://github.com/koide3/glim.git`
  - 依存のバージョン差分で起きる場合があるため、エラーログを共有いただければこちらで最小変更で対処します。

- 設定や地図保存:
  - `slam/glim/config` にホスト側の設定を置くと、コンテナから `/glim/config` として参照されます。
  - 生成物の保存先が指定できる場合は `/glim/maps` を使うと、ホストの `slam/glim/maps` に出力されます。

## カスタマイズ

- RViz2のプリセット設定を使いたい場合は、任意の `.rviz`/`.rviz2` を配置し、
  composeの `command` を `rviz2 -d /glim/config/your_config.rviz` のように変更してください。
- GPUを使った描画を安定させたい場合は、composeにデバイス要求（`--gpus all` 相当）を追加してください。
