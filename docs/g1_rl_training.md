# Unitree G1 強化学習ガイド<br>（公式 unitree_rl_lab）構築・学習・推論

本ドキュメントは、ヒューマノイド **Unitree G1** の歩行制御ポリシーを、Unitree 公式の強化学習リポジトリ **`unitree_rl_lab`** を用いて **Isaac Lab 上で学習（train）→ 推論（play）→ 実機/別シミュレータへの展開（sim2sim / sim2real）** まで行うための手順をまとめたものです。

`unitree_rl_lab` は Unitree Robotics が公開する公式リポジトリで、**Go2 / H1 / G1（29自由度）** に対応し、NVIDIA **Isaac Lab** と強化学習ライブラリ **`rsl_rl`** を基盤としています。学習アルゴリズムは **PPO（Proximal Policy Optimization）** で、ロボットの速度指令（前後・左右・旋回）に追従して歩行するポリシーを獲得します。

本ガイドの手順は、後掲の[検証環境](#a)（RTX 5090 Laptop / Isaac Sim 5.0 / Isaac Lab 2.2）で **実際に学習と推論が動作することを確認済み**です。

> 本ガイドのソフトウェアスタックは、ナビゲーション環境ガイド（`nav2_environment_setup`）と同じ Python 仮想環境 **`env_isaaclab_2.2`（Python 3.11）** を前提とします。既存環境を流用せず新規に構築する場合は、Isaac Sim 5.0 / Isaac Lab 2.2 を公式手順で導入したうえで本ガイドの第3部から進めてください。

<hr class="section-break" />

## はじめに

### 対象読者

Isaac Sim / Isaac Lab が導入済みのワークステーションで、Unitree G1 の歩行ポリシーを強化学習で獲得・検証したい技術担当者を想定します。Linux のコマンドライン、`git`、`pip`、Python 仮想環境（venv / conda）の基本操作を理解されている読者を前提とします。強化学習や Isaac Lab の内部実装の詳細知識は必須ではありません。

### 本ガイドの構成

| 部 | 内容 | 実施タイミング |
|---|---|---|
| 第1部 | 手法の概要（PPO・タスク定義・MDP） | 着手前の理解 |
| 第2部 | 環境前提（HW / SW / モデル資産） | 構築前の確認 |
| 第3部 | セットアップ（インストール・パス設定・タスク確認） | 一度だけ |
| 第4部 | 学習の実行（train） | ポリシー獲得 |
| 第5部 | 推論（play・エクスポート） | ポリシー検証 |
| 第6部 | 学習済みポリシーの展開（仮想SLAMナビへの組み込み・実機展開概要） | ポリシー活用 |
| 付録 | 検証環境・実測結果・主要パス・トラブルシュート | 随時 |

### この手法の位置づけ

Unitree G1 の歩行は、人手で制御則を書き下すのではなく、**シミュレータ内で多数のロボットを並列に動かし、報酬を最大化するように方策（ニューラルネットワーク）を学習**することで獲得します。本手法の特徴は次のとおりです。

- **大規模並列学習** — Isaac Lab（GPU 物理）上で数千体（既定 4096 体）の G1 を同時にシミュレートし、短時間で大量の経験を収集します。
- **Sim-to-Real 前提の設計** — 観測ノイズ・物理パラメータのランダム化（ドメインランダム化）・特権観測を用いた非対称 Actor-Critic により、実機へ転移しやすいポリシーを学習します。
- **公式デプロイ経路** — 学習済みポリシーは ONNX / TorchScript に書き出され、MuJoCo での sim2sim 検証を経て、`unitree_sdk2` 経由で実機 G1 に展開できます。

<hr class="section-break" />

## 第1部 手法の概要

### 1.1 全体パイプライン

強化学習による歩行獲得から展開までの流れは次のとおりです。本ガイドは **学習（Train）→ 推論（Play）** を詳説し、第6部で得られた `policy.onnx` の展開先（**仮想SLAMナビ**／実機）を示します。

```text
[1] Train (Isaac Lab)   4096体並列でPPO学習      → checkpoint (model_*.pt)
        │
[2] Play  (Isaac Lab)   学習済み方策を再生・検証  → policy.pt / policy.onnx
        │
        ├─ [3a] 仮想SLAMナビへ組み込み（policy.onnx 差し替え）★本ガイドの主目的
        │
        └─ [3b] Sim2Sim (MuJoCo) → Sim2Real (unitree_sdk2)  実機 G1 へ展開
```

### 1.2 学習アルゴリズム（PPO / rsl_rl）

学習は `rsl_rl` の On-Policy Runner による **PPO** で行います。G1・Go2・H1 で共通の基本設定（`BasePPORunnerCfg`）は次のとおりです。

| 項目 | 値 |
|---|---|
| アルゴリズム | PPO（Actor-Critic, on-policy） |
| 方策ネットワーク | MLP `[512, 256, 128]`・活性化 `ELU` |
| 価値ネットワーク | MLP `[512, 256, 128]`・活性化 `ELU` |
| 1イテレーションのロールアウト | `num_steps_per_env = 24` |
| 学習率 | `1.0e-3`（adaptive、`desired_kl = 0.01`） |
| 割引率 γ / GAE λ | `0.99` / `0.95` |
| クリップ幅 | `0.2`（value loss もクリップ） |
| エントロピー係数 | `0.01` |
| 学習エポック / ミニバッチ | `5` / `4` |
| 既定イテレーション数 | `50000`（`save_interval = 100`） |

### 1.3 タスク定義（速度追従ロコモーション）

タスク `Unitree-G1-29dof-Velocity` は、ランダムに与えられる速度指令（前後・左右・旋回）に追従して安定歩行することを目的とした MDP として定義されます。Isaac Lab の Manager ベース環境（`ManagerBasedRLEnv`）で構成され、主要な要素は次のとおりです。

| 要素 | 内容 |
|---|---|
| 制御周期 | 物理 `dt = 0.005s`（200Hz）・`decimation = 4` → **制御 50Hz** |
| エピソード長 | `20.0s` |
| 並列環境数 | 既定 `4096`（学習）/ `32`（再生） |
| 行動 | **29関節の目標位置**（`scale = 0.25`・既定姿勢オフセット付き） |
| 速度指令 | カリキュラムで段階的に拡大（最終 `lin_x ∈ [-0.5, 1.0]`, `lin_y ∈ [-0.3, 0.3]`, `yaw ∈ [-0.2, 0.2]`） |
| 報酬 | 速度追従（`track_lin_vel_xy`・`track_ang_vel_z`）＋生存報酬と、各種ペナルティ（関節速度/加速度・行動変化率・エネルギー・姿勢・接地・歩容・足滑り 等） |
| 終了条件 | 転倒（胴体の異常姿勢）・接触違反 等 |
| カリキュラム | 地形レベル＋速度指令レベルを学習進度に応じて段階的に難化 |

> **チューニング（旋回・その場回頭性能を上げたい場合）**: 速度指令の範囲と報酬の重みは `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/g1_29dof/velocity_env_cfg.py` で調整します。
>
> - **旋回を速くする**: `CommandsCfg.base_velocity` の `limit_ranges` にある `ang_vel_z`（既定 `(-0.2, 0.2)` rad/s ≒ ±11°/秒）を、例えば `(-1.0, 1.0)`（≒ ±57°/秒）へ拡大。必要に応じて初期カリキュラムの `ranges` 側の `ang_vel_z` も広げる。
> - **旋回追従を重視する**: `RewardsCfg.track_ang_vel_z` の `weight`（既定 `0.5`）を `0.75` 程度へ引き上げる。
> - **前進性能は維持される**: `lin_vel_x` の範囲と `track_lin_vel_xy`（`weight=1.0`）を据え置けば、前進を最優先したまま旋回だけ強化できます。
>
> いずれも **変更後は再学習が必要**です（既存の学習済みポリシーは旧設定の上限のままのため）。

### 1.4 観測と非対称 Actor-Critic

本手法は **方策（Actor）と価値関数（Critic）で異なる観測**を用いる *非対称 Actor-Critic* を採用します。Actor は実機でも得られる情報のみを観測し、Critic は学習時のみ利用できる特権情報（胴体線速度）を追加で観測することで、転移性と学習効率を両立します。いずれも直近 **5 ステップの履歴**を連結して入力します。

| 観測（Actor / 方策） | 観測（Critic / 価値・特権） |
|---|---|
| 胴体角速度・重力方向・速度指令 | 左記すべて |
| 関節相対位置・関節相対速度 | ＋ **胴体線速度**（特権情報） |
| 直前の行動 | ― |
| ※観測ノイズ（corruption）あり | ※ノイズなし |

### 1.5 報酬の定義（ファイルと数式）

報酬は **「どの項を・どの重みで使うか（構成）」** と **「各項の数式（実装）」** が別ファイルに分かれています。数式まで追う場合は下表のファイルを参照してください。ベースパス：`unitree/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/`

| 役割 | ファイル | 内容 |
|---|---|---|
| 報酬の構成・重み（G1-29dof） | `robots/g1/g1_29dof/velocity_env_cfg.py` の `class RewardsCfg`（238 行〜） | 使用する報酬項とその `weight`・パラメータ |
| 報酬の数式（独自項の実体） | `mdp/rewards.py` | `energy`・`foot_clearance_reward`・`feet_gait` 等の torch 実装 |
| 報酬の数式（標準項の実体） | Isaac Lab 組み込み `isaaclab/envs/mdp/rewards.py` | `track_lin_vel_xy_yaw_frame_exp`・`lin_vel_z_l2` 等。`RewardsCfg` から `mdp.xxx` として参照 |

**報酬項一覧（G1-29dof Velocity ／ `RewardsCfg`）** — 正の重み＝追従・生存を促進、負の重み＝ペナルティ。v＝胴体線速度、ω＝胴体角速度、g＝重力方向（胴体座標）、q＝関節位置、τ＝関節トルク、‖·‖＝ノルム。

| 項 | 重み | 数式（所在） |
|---|---|---|
| track_lin_vel_xy | +1.0 | exp(−‖v_xy^cmd − v_xy‖² / σ²), σ²=0.25（水平速度追従／標準） |
| track_ang_vel_z | +0.5 | exp(−(ω_z^cmd − ω_z)² / σ²), σ²=0.25（旋回速度追従／標準） |
| alive | +0.15 | 生存 1 ステップにつき +1（標準 is_alive） |
| base_linear_velocity | −2.0 | v_z²（上下動の抑制／標準 lin_vel_z_l2） |
| base_angular_velocity | −0.05 | ω_x² + ω_y²（ロール/ピッチ回転の抑制／標準 ang_vel_xy_l2） |
| joint_vel | −0.001 | Σ q̇²（標準 joint_vel_l2） |
| joint_acc | −2.5e-7 | Σ q̈²（標準 joint_acc_l2） |
| action_rate | −0.05 | Σ (a_t − a_t−1)²（行動の急変抑制／標準 action_rate_l2） |
| dof_pos_limits | −5.0 | 関節可動域の超過量（標準 joint_pos_limits） |
| energy | −2e-5 | Σ abs(q̇)·abs(τ)（消費エネルギー／**独自** rewards.py:energy） |
| joint_deviation_arms / waists / legs | −0.1 / −1 / −1 | Σ abs(q − q_default)（対象関節別／標準 joint_deviation_l1） |
| flat_orientation_l2 | −5.0 | g_x² + g_y²（胴体の傾き／標準） |
| base_height | −10 | (h − 0.78)²（目標胴体高 0.78 m／標準 base_height_l2） |
| gait | +0.5 | 接地が目標歩容（周期 0.8 s・左右位相 0 / 0.5）に一致（**独自** rewards.py:feet_gait） |
| feet_slide | −0.2 | 接地中の足の滑り速度（標準 feet_slide） |
| feet_clearance | +1.0 | exp(−Σ (z_foot − 0.1)²·tanh(2‖v_foot‖) / 0.05)（遊脚の足上げ高 0.1 m／**独自** foot_clearance_reward） |

> **重みの調整**は `velocity_env_cfg.py` の `RewardsCfg` で行います（§1.3 の旋回チューニング注記も参照）。各報酬項の学習中の寄与は TensorBoard の `Episode_Reward/*`（§4.3）で確認できます。

<hr class="section-break" />

## 第2部 環境前提

### 2.1 ハードウェア要件

| 項目 | 要件 |
|---|---|
| GPU | NVIDIA RTX 系（**24GB VRAM 推奨**）。本ガイドは RTX 5090 Laptop (24GB) で検証 |
| GPU アーキ | Blackwell (sm_120) を含む。torch は対応版（cu128）が必要 |
| システムメモリ | 32GB 以上推奨 |
| ディスク | Isaac Sim / Isaac Lab とログ用に十分な空き |

> **注意（Blackwell GPU）**: RTX 50 系（sm_120）では、CUDA 12.8 対応の PyTorch（`torch 2.7.0+cu128` など）が必須です。古い CUDA ビルドの torch では GPU を認識できません。本検証環境は `torch 2.7.0+cu128` で動作します。

### 2.2 ソフトウェアスタック

| ソフトウェア | バージョン（検証） |
|---|---|
| OS | Ubuntu 22.04 |
| Python | 3.11 |
| Isaac Sim | 5.0.0 |
| Isaac Lab | 2.2（`isaaclab` 0.44.x 系） |
| PyTorch | 2.7.0+cu128（CUDA 12.8） |
| rsl_rl | `rsl_rl_lib` 2.3.x |
| Python 仮想環境 | `env_isaaclab_2.2` |

### 2.3 ロボットモデル資産（unitree_model）

G1 の USD モデルは Unitree 公式データセット **`unitree_model`** から取得します（本リポジトリでは `unitree/unitree_model` に配置済み）。G1（29自由度）の USD は次のパスを使用します。

```text
unitree/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd
```

新規取得する場合は Hugging Face から取得します（フォルダ構成を保持）。

```bash
git lfs install
git clone https://huggingface.co/datasets/unitreerobotics/unitree_model
```

<hr class="section-break" />

## 第3部 セットアップ

### 3.1 リポジトリ配置

`unitree_rl_lab` は本リポジトリ内 `unitree/unitree_rl_lab` に含まれます。Isaac Lab 本体とは別ディレクトリに置く構成です。

```bash
cd ~/work/humanoid-isaac-ros2/unitree/unitree_rl_lab
```

### 3.2 unitree_rl_lab のインストール（editable）

Isaac Lab を導入済みの Python 環境を有効化し、`unitree_rl_lab` を editable インストールします。追加依存は `psutil` のみで、Isaac Sim / PyTorch スタックには影響しません。

```bash
source ~/env_isaaclab_2.2/bin/activate
cd ~/work/humanoid-isaac-ros2/unitree/unitree_rl_lab
python -m pip install -e source/unitree_rl_lab
```

インストール確認:

```bash
python -c "import unitree_rl_lab; print('unitree_rl_lab OK')"
```

### 3.3 モデルパス設定（UNITREE_MODEL_DIR）

ロボット資産のパスは `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` の `UNITREE_MODEL_DIR` で決まります。本リポジトリ構成では相対パスが `unitree/unitree_model` を正しく指すよう設定済みです。別の場所に `unitree_model` を置いた場合のみ、絶対パスへ書き換えてください。

```python
# unitree.py（抜粋）
UNITREE_MODEL_DIR = os.path.abspath(
    os.path.join(current_dir, "../../../../../../unitree_model")
)
# 例: 絶対パスで指定する場合
# UNITREE_MODEL_DIR = "/home/<user>/.../unitree/unitree_model"
```

### 3.4 登録タスクの確認（list_envs）

タスクが正しく登録されているかを確認します。

```bash
python scripts/list_envs.py
```

次のように G1 を含む3タスクが表示されれば成功です。

```text
| S. No. | Task Name                 | Entry Point                     | Config        |
|   1    | Unitree-G1-29dof-Velocity | isaaclab.envs:ManagerBasedRLEnv | ...g1...       |
|   2    | Unitree-Go2-Velocity      | isaaclab.envs:ManagerBasedRLEnv | ...go2...      |
|   3    | Unitree-H1-Velocity       | isaaclab.envs:ManagerBasedRLEnv | ...h1...       |
```

> **注意（出力が途中で切れる場合）**: Isaac Sim はアプリ終了時にプロセスを即時終了するため、Python の標準出力バッファがフラッシュされず、テーブルが表示されないことがあります。その場合は下記のように **アンバッファ実行**してください（本ガイドの実行例ではこの指定を推奨）。

```bash
PYTHONUNBUFFERED=1 python -u scripts/list_envs.py
# または（libexpat 対策も兼ねる）: ./scripts/rsl_rl/run_rl.sh list_envs.py
```

<hr class="section-break" />

## 第4部 学習の実行

### 4.1 学習コマンド

ヘッドレス（GUI なし）で G1 の速度追従ポリシーを学習します。

```bash
source ~/env_isaaclab_2.2/bin/activate
cd ~/work/humanoid-isaac-ros2/unitree/unitree_rl_lab

# ランチャ経由（推奨）。libexpat 版ズレ対策・アンバッファ実行を自動設定
./scripts/rsl_rl/run_rl.sh train.py \
    --headless \
    --task Unitree-G1-29dof-Velocity \
    --num_envs 4096 \
    --max_iterations 1000
```

> **`run_rl.sh`（ランチャ）について**: `train.py` / `play.py` を venv の python で**直接**実行すると、Isaac Sim が差し込む古い `libexpat` のために `pyexpat` 由来のエラー（`undefined symbol` 等）で起動に失敗することがあります（→ [付録 C](#c)）。`run_rl.sh` はその対策（`LD_PRELOAD`）と `PYTHONUNBUFFERED=1` を自動で行うラッパです（`train.py`・`play.py`・`list_envs.py` 共用、引数はそのまま渡せます）。`activate` は Python の切替だけで `libexpat` の読み込み順には関与しないため、直接実行する場合は付録 C の `LD_PRELOAD` を先に設定してください。

### 4.2 主要オプション

| オプション | 説明 | 既定 |
|---|---|---|
| `--task` | タスク名（`Unitree-G1-29dof-Velocity`） | ― |
| `--headless` | GUI を表示せず学習（高速・推奨） | 無効 |
| `--num_envs` | 並列環境数。VRAM に応じて調整 | 4096 |
| `--max_iterations` | 学習イテレーション数 | 50000 |
| `--seed` | 乱数シード | 設定値 |
| `--video` | 学習中の動画を記録（`--video_length` / `--video_interval` 併用） | 無効 |

> **VRAM 調整**: 24GB GPU では `--num_envs 4096`（既定）が動作します。VRAM が不足する場合は `--num_envs 2048` などに下げてください。他プロセスと GPU を共有している場合も同様です。

### 4.3 学習中の見方

学習が始まると、イテレーションごとに収集速度（steps/s）・平均報酬・各報酬項の内訳が表示されます。

```text
 Learning iteration 53/1000
 Computation: 23930 steps/s (collection: 3.838s, learning 0.270s)
 Mean reward: -0.76
   Episode_Reward/track_lin_vel_xy: ...
   Episode_Reward/track_ang_vel_z:  ...
```

`Mean reward`（平均報酬）が学習の進行とともに上昇していけば学習は順調です。TensorBoard でも報酬曲線を確認できます。

```bash
tensorboard --logdir logs/rsl_rl/unitree_g1_29dof_velocity
```

> **TensorBoard でのタグ名**: ターミナルに表示される `Mean reward` は、TensorBoard では **`Train/mean_reward`**（`Train` グループ内）という名前のグラフとして表示されます（「Mean reward」という名前のグラフはありません）。あわせて、各報酬項の内訳は **`Episode_Reward/*`**（例: `Episode_Reward/track_lin_vel_xy`、`Episode_Reward/track_ang_vel_z`）、エピソード長は `Train/mean_episode_length` で確認できます。

### 4.4 出力物

ログとチェックポイントはタイムスタンプ付きディレクトリに保存されます。

```text
logs/rsl_rl/unitree_g1_29dof_velocity/<YYYY-MM-DD_HH-MM-SS>/
├── model_0.pt          # 初期チェックポイント
├── model_100.pt        # save_interval(=100)ごと
├── model_<最終>.pt      # 学習終了時
├── params/             # 解決済みの環境・エージェント設定
├── git/                # 実行時のコードのgit diff（再現用）
└── events.out.tfevents.*  # TensorBoard ログ
```

<hr class="section-break" />

## 第5部 推論（学習済みポリシーの確認）

### 5.1 play コマンド

> **補足（バージョン互換）**: 検証環境のような **Isaac Lab 2.2 / rsl_rl 2.x** 系で `play.py` を動かすには 3 か所の互換修正が必要ですが、**本リポジトリには反映済みで操作は不要**です。Isaac Sim 5.1（Isaac Lab 2.3 / rsl_rl 3.x）系では無修正で動作します。修正内容の詳細は [付録 D](#d) を参照してください。

学習済みポリシーを再生して挙動を確認します。`--checkpoint` を省略すると最新の学習結果が自動で読み込まれます。

```bash
# 最新の学習結果を自動選択して再生（GUI 表示）。ランチャ経由（§4.1 参照）
./scripts/rsl_rl/run_rl.sh play.py --task Unitree-G1-29dof-Velocity --num_envs 32

# チェックポイントを明示指定する場合
./scripts/rsl_rl/run_rl.sh play.py --task Unitree-G1-29dof-Velocity --num_envs 32 \
    --checkpoint logs/rsl_rl/unitree_g1_29dof_velocity/<run>/model_<iter>.pt
```

GUI が使えない環境では、ヘッドレスで動画を記録できます。

```bash
./scripts/rsl_rl/run_rl.sh play.py \
    --headless --task Unitree-G1-29dof-Velocity --num_envs 32 \
    --video --video_length 200 \
    --checkpoint logs/.../model_<iter>.pt
```

### 5.2 ポリシーのエクスポート（jit / onnx）

`play.py` は実行時に学習済み方策を **TorchScript（`policy.pt`）と ONNX（`policy.onnx`）** に自動で書き出します。これらが sim2sim / sim2real で使用する成果物です。

```text
logs/rsl_rl/unitree_g1_29dof_velocity/<run>/exported/
├── policy.pt      # TorchScript
└── policy.onnx    # ONNX
```

### 5.3 動画記録

`--video` 指定時、再生動画は `<run>/videos/play/` 配下に `.mp4` で保存されます。GUI のない検証環境でも歩容を目視確認できます。

<video controls playsinline width="100%" style="max-width:640px; display:block; margin:1.2em auto; border:1px solid var(--border); border-radius:6px; box-shadow:var(--shadow); background:var(--bg-card);">
  <source src="assets/g1_rl_25000.webm" type="video/webm">
  お使いのブラウザは動画再生に対応していません。<code>docs/assets/g1_rl_25000.webm</code> を直接ご覧ください。
</video>

*約 25,000 イテレーション学習後（報酬が約 42 で収束）の `Unitree-G1-29dof-Velocity` ポリシーを `play.py` で再生した様子。頭上の緑／青のマーカーは与えられた速度指令を表し、G1 が転倒せず速度指令に追従して歩行する。*

<hr class="section-break" />

## 第6部 学習済みポリシーの展開

学習・エクスポートした `policy.onnx` を、実際に使う先へ展開します。本ガイドの主目的は **6.1 の仮想 SLAM ナビへの組み込み**です。実機への展開（6.2）は概要のみ示します。

### 6.1 仮想 SLAM ナビへの組み込み（policy.onnx の差し替え）

> **ステータス**: 入出力仕様の互換性（観測 480 / 行動 29・同一ネットワーク構造）とナビ側の読み込み機構は**確認済み**です。学習済みポリシーを実際に差し替えてナビで歩行させる**エンドツーエンドの動作確認は、十分に学習が進んだ段階で実施予定**です（本手順はその互換性確認に基づきます）。

本リポジトリの仮想ナビ（Isaac Sim 上で LiDAR・SLAM・Nav2 と連携して G1 を歩かせる構成）は、G1 の歩行を **ONNX ポリシー**で駆動しています。ロコモーション実行スクリプト `HumanoidPoC/scripts/environments/locomotion/g1/onnx_locomotion_g1.py`（タスク `Isaac-Velocity-Rough-RtxLidar-G1-v0`）が、次のファイルを読み込みます。

```text
HumanoidPoC/scripts/environments/locomotion/g1/models/policy.onnx
```

このナビ用ポリシーは、本ガイドの `Unitree-G1-29dof-Velocity` と **同一の観測・行動仕様**（観測 480 次元＝96 項目 × 履歴 5、行動 29 関節）で作られています。したがって本ガイドで学習・エクスポートした `policy.onnx` は、**ファイルを差し替えるだけ**で組み込めます（Nav2 からの速度指令は、ナビ側スクリプトが観測の `velocity_commands` 枠へ自動で注入します）。

**手順:**

**(1) 組み込みたいポリシーを `play.py` で書き出す。** どの時点のポリシーになるかは、ここで指定する `--checkpoint` で決まります（省略時はそのrunの最新 checkpoint。学習中の一時的なスパイク直後は避け、安定区間の `model_<iter>.pt` を選ぶのが安全）。

```bash
cd ~/work/humanoid-isaac-ros2/unitree/unitree_rl_lab

# 特定の checkpoint を書き出す例（<run>=日時フォルダ, <iter>=保存番号）
python scripts/rsl_rl/play.py --task Unitree-G1-29dof-Velocity \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_velocity/<run>/model_<iter>.pt
# → 同じ run の exported/policy.onnx が、その checkpoint の内容で生成・更新される
```

**(2) 書き出した `policy.onnx` をナビ用の場所へ差し替える。** `<run>` は学習ごとに作られる日時フォルダ（例 `2026-06-23_15-58-29`）です。**run は ①最新を自動取得／②特定を手動指定 のどちらでも**選べます。`cd` でパスが崩れないよう**絶対パス**で扱います。

```bash
cd ~/work/humanoid-isaac-ros2/unitree/unitree_rl_lab

# --- run の選択（① か ② のどちらか）---
RUN=$(ls -dt logs/rsl_rl/unitree_g1_29dof_velocity/*/ | head -1)   # ① 最新を自動取得
# RUN=logs/rsl_rl/unitree_g1_29dof_velocity/2026-06-23_15-58-29/   # ② 特定runを指定(末尾 / 必須)

SRC=$(realpath "${RUN}exported/policy.onnx")   # 学習済みポリシーの絶対パス
ls -l "$SRC"                                    # 存在確認（無ければ (1) の play.py を実行）

# --- ナビ用ポリシーをバックアップ → 差し替え ---
NAV=~/work/humanoid-isaac-ros2/HumanoidPoC/scripts/environments/locomotion/g1/models
cp "$NAV/policy.onnx" "$NAV/policy.onnx.bak"    # 既存をバックアップ（元に戻せる）
cp "$SRC"            "$NAV/policy.onnx"          # 学習済みへ差し替え
```

**(3) 仮想ナビ（ロコモーション）を通常の手順で起動**し、G1 が新ポリシーで歩行することを確認します（`onnx_locomotion_g1.py`）。元に戻すには `cp "$NAV/policy.onnx.bak" "$NAV/policy.onnx"` を実行します。

> **注意（互換）**: 差し替えるポリシーは観測・行動仕様が一致している必要があります。本ガイドの `Unitree-G1-29dof-Velocity` で学習したものは一致します（入出力とも同一）。観測項目を変えた独自タスクの方策は次元が合わず動作しません。
>
> **旋回性能**: 仮想 SLAM でその場回頭を機敏にしたい場合は、第1.3部のチューニング（`ang_vel_z` の範囲拡大・`track_ang_vel_z` の重み増）を反映して学習し直したポリシーを差し替えてください。

### 6.2 実機への展開（sim2sim / sim2real）概要

実機 G1 へ展開する場合は、`policy.onnx` を MuJoCo で sim2sim 検証してから sim2real します（**仮想ナビが目的の場合は不要**。実機展開時のみ）。詳細は `unitree_rl_lab` の README を参照してください。

```bash
# 依存と g1_ctrl のビルド
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build && cmake .. -DBUILD_EXAMPLES=OFF && sudo make install
cd unitree_rl_lab/deploy/robots/g1_29dof && mkdir build && cd build && cmake .. && make
```

- **Sim2Sim**: `unitree_mujoco` を導入し、`g1_ctrl` でポリシーを再生（MuJoCo で挙動確認）。
- **Sim2Real**: 実機の純正制御を停止し、`./g1_ctrl --network eth0` で起動。

> **注意（安全）**: 実機展開は転倒・暴れによる事故の危険があります。吊り下げ・安全柵・非常停止を用意し、メーカーの安全手順に従ってください。

<hr class="section-break" />

## 付録 A 検証環境・実測結果 {#a}

本ガイドの手順は次の環境で **学習・推論の動作を確認済み**です。

| 項目 | 内容 |
|---|---|
| GPU | NVIDIA GeForce RTX 5090 Laptop GPU (24GB) |
| ドライバ | 580.159.03 |
| PyTorch | 2.7.0+cu128（sm_120 対応） |
| Isaac Sim / Lab | 5.0.0 / 2.2 |
| Python 環境 | `env_isaaclab_2.2`（Python 3.11） |
| 学習タスク | `Unitree-G1-29dof-Velocity` |
| 学習スループット | 約 **23,000〜24,000 steps/s**（`--num_envs 4096`・headless） |
| 平均報酬の推移 | 0 iter: `-0.88` → 100 iter: `-0.12` → 200 iter: `+5.6` → 1000 iter: `約 +7.0`（収束） |
| 学習時間 | 1000 iter で約 **80 分**（RTX 5090 Laptop・`--num_envs 4096`） |
| 推論（play） | 学習済み checkpoint の読み込み・再生・動画記録（mp4）まで動作確認 |
| エクスポート | `policy.pt`（TorchScript）/ `policy.onnx` の生成を確認 |

## 付録 B 主要パス {#b}

| 用途 | パス |
|---|---|
| unitree_rl_lab | `unitree/unitree_rl_lab` |
| 学習・推論スクリプト | `unitree/unitree_rl_lab/scripts/rsl_rl/{train,play}.py` |
| タスク一覧スクリプト | `unitree/unitree_rl_lab/scripts/list_envs.py` |
| ロボット資産設定 | `.../source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` |
| G1 タスク定義 | `.../tasks/locomotion/robots/g1/g1_29dof/velocity_env_cfg.py` |
| PPO 設定 | `.../tasks/locomotion/agents/rsl_rl_ppo_cfg.py` |
| G1 USD モデル | `unitree/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd` |
| 学習ログ | `unitree/unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_velocity/` |

## 付録 C トラブルシューティング {#c}

| 症状 | 原因 / 対処 |
|---|---|
| `list_envs` でテーブルが表示されない | アプリ終了時のバッファ未フラッシュ。`PYTHONUNBUFFERED=1 python -u ...`（または `run_rl.sh` 経由）で実行 |
| `train.py` / `play.py` 起動時に `pyexpat` / `libexpat` のエラー（`undefined symbol`・バージョン不一致 等） | conda 由来 venv の `libexpat` と Isaac が差し込む古い `libexpat` の版ズレ。**`run_rl.sh` 経由で実行**（自動で `LD_PRELOAD` 設定）。直接実行時は先に `export LD_PRELOAD="$(~/env_isaaclab_2.2/bin/python -c 'import sys; print(sys.base_prefix)')/lib/libexpat.so.1"`。`activate` だけでは解決しない（§4.1 参照） |
| `play.py` で `ImportError: ...DistillationRunner` | rsl_rl 2.x 環境。付録 D の修正①（import の任意化）を適用（リポジトリ反映済み） |
| `play.py` で `AttributeError: ...'class_name'` | isaaclab_rl < 0.4 環境。付録 D の修正②（`getattr` フォールバック）を適用（リポジトリ反映済み） |
| `play.py` で `TypeError: linear(): ... not tuple` | 2.2 系の `get_observations()` がタプルを返す。付録 D の修正③（obs アンパック）を適用（リポジトリ反映済み） |
| `Warp CUDA error: ... cuDeviceGetUuid` の警告 | 軽微な警告で動作に影響なし（Warp が機能をフォールバック）。無視してよい |
| `Disabling key-value database because another kit process is locking it` | 別の Isaac Sim プロセスが起動中。同時実行時に出る警告で、致命的ではない |
| GPU が認識されない / torch で CUDA 利用不可 | RTX 50 系では CUDA 12.8 対応 torch（`+cu128`）が必須。古いビルドを置き換える |
| 学習中に VRAM 不足（out of memory） | `--num_envs` を 2048 などに下げる。他プロセスと GPU 共有時も同様 |
| G1 の USD が見つからない | `UNITREE_MODEL_DIR` を確認。`unitree_model` の配置とパスを合わせる |

## 付録 D Isaac Sim 5.1 系で実施したい場合 {#d}

本ガイドは、検証済みの **Isaac Sim 5.0 / Isaac Lab 2.2**（仮想環境 `env_isaaclab_2.2`）を既定の前提としています。この構成では、後述の互換修正が `play.py` に**反映済み**のため、利用者は**特別な対応をせずそのまま**学習・推論を実行できます（本文の手順どおりでOK）。

一方、`unitree_rl_lab` は公式には **Isaac Sim 5.1 / Isaac Lab 2.3 / rsl_rl 3.x** を対象としています。**あえて 5.1 系で実施したい場合**は、次の点に留意してください。

- **環境構築**: Isaac Sim 5.1 + Isaac Lab 2.3 を別途導入する（本文 第2部のバージョン表記 5.0 / 2.2 を 5.1 / 2.3、rsl_rl を 3.x と読み替え）。
- **学習・推論の手順・タスク名・コマンドは本ガイドと同一**。`train.py` / `play.py` ともそのまま使えます。
- **`play.py` は無修正で動作**します（元々 rsl_rl 3.x 前提で書かれているため）。下記の互換修正は 5.1 系では**不要**ですが、入っていても**無害に素通り**するので、5.0 / 5.1 のどちらでも同じ `play.py` が使えます。

### 参考：5.0 / 2.2 系で `play.py` を動かすための互換修正（反映済み）

検証環境（Isaac Lab 2.2 / rsl_rl 2.x）で `play.py` を起動するには、次の 3 か所の修正が必要でした（`train.py` は無修正で動作）。**本リポジトリには適用済み**です。5.1 系では不要・無害です。

**① `DistillationRunner` の import を任意化**（`DistillationRunner` は rsl_rl 3.x の機能）:

```python
# 変更前: from rsl_rl.runners import DistillationRunner, OnPolicyRunner
from rsl_rl.runners import OnPolicyRunner
try:
    from rsl_rl.runners import DistillationRunner  # rsl_rl >= 3.x のみ
except ImportError:
    DistillationRunner = None
```

**② Runner 選択を `getattr` フォールバックに**（runner cfg の `class_name` は isaaclab_rl 0.4 以降のみ存在）:

```python
# 変更前: if agent_cfg.class_name == "OnPolicyRunner":
runner_class_name = getattr(agent_cfg, "class_name", "OnPolicyRunner")
if runner_class_name == "OnPolicyRunner":
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
elif runner_class_name == "DistillationRunner":
    runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
```

**③ 初期観測のアンパック**（2.2 系の `get_observations()` は `(obs, extras)` のタプルを返す）:

```python
obs = env.get_observations()
if isinstance(obs, tuple):   # isaaclab_rl < 0.4 は (obs, extras) を返す
    obs = obs[0]
```
