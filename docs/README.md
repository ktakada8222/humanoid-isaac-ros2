# ドキュメント一覧（docs/）

G1 ヒューマノイド（Isaac Sim 5.0 + ROS 2 Humble + Nav2）の手順書一式です。
各ドキュメントは **`.md`（編集用ソース）** と **`.html`（閲覧用・体裁付き）** のペアで提供しています。

## `.md` と `.html` の違い
- **`.html`** … ブラウザで開く完成版マニュアル。画像・動画・図を埋め込んだ **単一ファイル完結**（オフライン閲覧可）。**読むだけならこちら**を開けば OK です。
- **`.md`** … 同じ内容の Markdown ソース。テキストエディタや GitHub で閲覧・**編集**するための原稿。`.html` はここから生成されます。
- **`build_*.py`** … `.md` → `.html` を生成するビルダー（内容を編集したあとの再生成用）。

## ドキュメント
| ドキュメント（`.md` / `.html`） | 内容 | 読むタイミング |
|---|---|---|
| **`nav2_environment_setup`** | **メイン手順書**。事前準備 → ソース受領後の構築 → コンテナ基盤 → 地図作成 → ナビ実行。付録にプロジェクト構成（フォルダツリー）・主要パス・GIMP・Tilix。 | まず最初に通す |
| **`navigation_run`** | **環境構築済みからの起動ガイド**。Isaac 起動 → hdl → laserscan → nav2 → RViz の運用手順と地図の選択・差し替え。 | 構築後、日々ナビを動かすとき |
| **`nav2_parameters`** | **Nav2 パラメータ設定ガイド**。`nav2_params_*.yaml` の各値（速度上限・コストマップ・障害物回避・Behavior Tree 等）の意味と本 PoC の設定、ワークスペースのビルド方法。 | パラメータを調整するとき |
| **`g1_rl_training`** | **強化学習ガイド**。公式 `unitree_rl_lab` による G1 歩行ポリシーの学習・推論・展開。報酬の定義（§1.5）、学習/推論ランチャ `run_rl.sh`。 | 歩行ポリシーを学習・差し替えるとき |

## ビルダー対応表（`.md` → `.html` 再生成）
| ドキュメント | ビルダー |
|---|---|
| `nav2_environment_setup` | `build_doc_html.py` |
| `navigation_run` | `build_navigation_run_html.py` |
| `nav2_parameters` | `build_nav2_params_html.py` |
| `g1_rl_training` | `build_g1_rl_html.py` |

再生成の例（リポジトリ直下から実行）:

```bash
~/env_isaaclab_2.2/bin/python docs/build_doc_html.py     # nav2_environment_setup.md → .html
```

> ビルダーは `.md` から `.html` を作り直すだけです。画像・動画は `.html` に base64 で埋め込まれ、単一ファイルで完結します。

## その他のフォルダ
- **`assets/`** … 図・スクリーンショット・解説動画（`.html` には埋め込み済みのため、配布時に個別参照は不要）
