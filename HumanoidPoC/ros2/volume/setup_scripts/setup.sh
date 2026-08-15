#!/bin/bash
set -e

# 環境変数
ROS_DISTRO=${ROS_DISTRO:-humble}

sudo apt update

# ROS環境読み込み
source /opt/ros/$ROS_DISTRO/setup.bash

# ワークスペース作成
mkdir -p /ros2_ws/src
cd /ros2_ws/src

# navigation2 をクローン（存在しないときのみ）
if [ ! -d navigation2 ]; then
    git clone https://github.com/ros-planning/navigation2.git --branch $ROS_DISTRO navigation2
else
    echo "navigation2 already exists, skipping clone."
fi

# pcd2pgm をクローン（存在しないときのみ）
if [ ! -d pcd2pgm ]; then
    git clone https://github.com/LihanChen2004/pcd2pgm.git
else
    echo "pcd2pgm already exists, skipping clone."
fi

# hdl_localization関連をクローン（存在しないときのみ）
if [ ! -d ndt_omp ]; then
    git clone --recursive https://github.com/koide3/ndt_omp.git
else
    echo "ndt_omp already exists, skipping clone."
fi

if [ ! -d fast_gicp ]; then
    git clone --recursive https://github.com/SMRT-AIST/fast_gicp.git
else
    echo "fast_gicp already exists, skipping clone."
fi

if [ ! -d hdl_global_localization ]; then
    git clone --recursive -b humble https://github.com/koide3/hdl_global_localization.git
else
    echo "hdl_global_localization already exists, skipping clone."
fi

if [ ! -d hdl_localization ]; then
    git clone --recursive -b humble https://github.com/AkamisakaAtsuki/hdl_localization.git
else
    echo "hdl_localization already exists, skipping clone."
fi

if [ ! -d pcd_publisher ]; then
    git clone https://github.com/omerssid/pcd_publisher.git
else
    echo "pcd_publisher already exists, skipping clone."
fi

cd /ros2_ws

# rosdep の初期化（初回だけ）
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 依存関係インストール（不要なキーはスキップ）
sudo rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  --skip-keys="pcl_ros ament_cmake_clang_format"

# 念のため PCL と Eigen を明示的に入れる
sudo apt-get install -y libpcl-dev libeigen3-dev

# Turtlebot3 とシミュレーション
sudo apt install -y ros-${ROS_DISTRO}-turtlebot3 \
                    ros-${ROS_DISTRO}-turtlebot3-bringup \
                    ros-${ROS_DISTRO}-turtlebot3-simulations \
                    ros-${ROS_DISTRO}-pointcloud-to-laserscan

# CycloneDDS RMW 実装をインストール
sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
sudo apt install -y ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions

# ==== hdl_localization の依存パッケージ ====
sudo apt install -y \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev libpcl-dev \
    libtbb-dev libboost-all-dev \
    libopencv-dev python3-opencv

# ==== GTSAM のビルド ====
if [ ! -d /tmp/gtsam ]; then
    cd /tmp
    git clone https://github.com/borglab/gtsam.git -b 4.2.0
    cd gtsam && mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    cd /
else
    echo "GTSAM already exists, skipping build."
fi

# ビルド
cd /ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✅ Setup completed. Please run:"
echo "   source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "   source /ros2_ws/install/setup.bash"

# bashrc に RMW 実装を指定（重複防止）
if ! grep -q "RMW_IMPLEMENTATION" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
fi
# bashrc に環境変数を追加（重複防止つき）
if ! grep -q "TURTLEBOT3_MODEL" ~/.bashrc; then
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
fi
if ! grep -q "GAZEBO_MODEL_PATH" ~/.bashrc; then
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models" >> ~/.bashrc
fi
if ! grep -q "source /ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
fi
