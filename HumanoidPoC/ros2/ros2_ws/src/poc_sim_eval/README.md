# poc_launchのコンパイル
cd /ros2_ws
colcon build --packages-select poc_sim_eval
source install/setup.bash

ros2 run poc_sim_eval nav2_auto_eval --ros-args -p config_path:=/ros2_ws/src/poc_sim_eval/config/config.yaml

<!-- 
python3 nav2_auto_eval.py --ros-args -p config_path:=/ros2_ws/src/poc_sim_eval/config/config.yaml -->
