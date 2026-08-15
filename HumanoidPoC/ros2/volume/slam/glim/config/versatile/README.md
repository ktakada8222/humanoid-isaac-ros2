このディレクトリは、GLIM起動時の既定設定とRVizプリセットを格納します。

- GLIM構成は `config_path:=/glim/config/versatile` で参照されます。
- RVizは `rviz.rviz` を読み込みます。
- トピックは compose 側で `/velodyne/points`（点群）と `/velodyne/points_imu`（IMU）にリマップ済みです。

必要に応じて本ディレクトリに追加設定ファイル（YAML等）を置いてください。
