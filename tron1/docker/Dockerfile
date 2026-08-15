FROM ros:noetic-ros-base-focal

RUN apt-get update && apt-get install -y \
    cmake build-essential git python3-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN git clone --recurse https://github.com/limxdynamics/tron1-mujoco-sim.git

RUN apt-get update && apt-get install -y libeigen3-dev 

# SDK wheel
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        pip3 install tron1-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl; \
    elif [ "$ARCH" = "aarch64" ]; then \
        pip3 install tron1-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl; \
    else \
        echo "Unsupported arch: $ARCH" >&2; exit 2; \
    fi

# ここから bash -lc を使う
SHELL ["/bin/bash", "-lc"]

# ROS を読み込んでから CMake
RUN source /opt/ros/noetic/setup.bash && \
    cd tron1-mujoco-sim/limxsdk-lowlevel && \
    mkdir -p build && cd build && \
    cmake .. && make -j"$(nproc)"

WORKDIR /workspace/tron1-mujoco-sim/limxsdk-lowlevel/build/examples
ENTRYPOINT ["/bin/bash"]
