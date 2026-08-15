# 仮想環境の構築
conda create -n tron1-py312 python=3.12 -y
conda activate tron1-py312

echo 'export ROBOT_TYPE=PF_TRON1A' >> ~/.bashrc && source ~/.bashrc

# lowlevel apiのインストール
pip install tron1-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl

# シミュレータの軌道
python tron1-mujoco-sim/simulator.py 


# コントラーラの起動
```bash
tron@tron:~/humanoid-limx-oli-isaac-ros2/tron1/docker$ docker compose up
```
