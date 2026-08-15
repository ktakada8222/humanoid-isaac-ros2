#!/usr/bin/env bash
# unitree_rl_lab 用 学習/推論ランチャ
#
# 目的:
#   * venv(env_isaaclab_2.2) の python で対象スクリプトを実行する
#   * Isaac Sim が差し込む LD_LIBRARY_PATH により古いシステム libexpat が
#     優先され、conda 由来 venv の pyexpat が壊れる問題を、base_prefix の
#     libexpat.so.1 を LD_PRELOAD することで回避する（ガード付き＝在るときだけ）
#   * Isaac はアプリ終了時に即時終了し stdout をflushしないため、
#     PYTHONUNBUFFERED=1 でアンバッファ実行する
#
# 使い方:
#   ./scripts/rsl_rl/run_rl.sh train.py --headless --task Unitree-G1-29dof-Velocity --num_envs 4096
#   ./scripts/rsl_rl/run_rl.sh play.py  --task Unitree-G1-29dof-Velocity --num_envs 32
#   ./scripts/rsl_rl/run_rl.sh list_envs.py
#
# 上書き可能な環境変数: ISAAC_PY（venv の python。既定 ~/env_isaaclab_2.2/bin/python）
set -euo pipefail

ISAAC_PY="${ISAAC_PY:-$(command -v python)}"

# このスクリプトの位置から unitree_rl_lab ルートを解決し、そこで実行する
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_LAB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"   # scripts/rsl_rl/ -> unitree_rl_lab/
cd "$RL_LAB_ROOT"

# --- libexpat 版ズレ対策（ガード付き）--------------------------------------
# base_prefix(conda) の libexpat.so.1 が在るときだけ最優先で読ませる。無ければ no-op。
# 既に LD_PRELOAD が設定済みなら尊重して上書きしない。
if [ -z "${LD_PRELOAD:-}" ]; then
  BASE_PREFIX="$("$ISAAC_PY" -c 'import sys; print(sys.base_prefix)' 2>/dev/null || true)"
  if [ -n "$BASE_PREFIX" ] && [ -f "$BASE_PREFIX/lib/libexpat.so.1" ]; then
    export LD_PRELOAD="$BASE_PREFIX/lib/libexpat.so.1"
  fi
fi

export PYTHONUNBUFFERED=1

if [ "$#" -lt 1 ]; then
  echo "usage: $0 <script.py> [args...]" >&2
  echo "  e.g. $0 train.py --headless --task Unitree-G1-29dof-Velocity --num_envs 4096" >&2
  exit 2
fi

TARGET="$1"; shift
# train.py / play.py は scripts/rsl_rl/ 配下、list_envs.py は scripts/ 配下
if [ -f "scripts/rsl_rl/$TARGET" ]; then
  REL="scripts/rsl_rl/$TARGET"
elif [ -f "scripts/$TARGET" ]; then
  REL="scripts/$TARGET"
elif [ -f "$TARGET" ]; then
  REL="$TARGET"
else
  echo "script not found: $TARGET (looked in scripts/rsl_rl/ and scripts/)" >&2
  exit 2
fi

exec "$ISAAC_PY" -u "$REL" "$@"
