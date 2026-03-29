# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

ROS 2 Humble ベースの Artificial Potential Field (APF) 経路計画チュートリアルパッケージ。RViz2 上の Interactive Markers で障害物を配置・操作しながら、APF による経路計画をリアルタイムに確認できる教育用ツール。

## ビルド・起動

```bash
# ビルド（ros2_ws/src にパッケージを配置した上で）
cd ~/ros2_ws && colcon build --packages-select apf_pathplanner_tutorial

# 起動（RViz2 + 経路計画ノード）
ros2 launch apf_pathplanner_tutorial apf_pathplanner_tutorial.launch.py
```

Docker 環境の場合：
```bash
docker exec potbot bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && cd /root/ros2_ws && colcon build --packages-select apf_pathplanner_tutorial"
```

依存パッケージ `potbot_lib`、`potbot_ros` が ros2_ws に必要。

## アーキテクチャ

### メインノード (`apf_pathplanner_tutorial`)

`src/apf_pathplanner_tutorial.cpp` — `MarkerPathPlanner` クラス（`rclcpp::Node` 継承）がすべてを統括：

- **potbot_lib** の `ArtificialPotentialField` と `APFPathPlanner` を直接使用して経路計画を実行
- **Interactive Markers** で障害物（CUBE/SPHERE）を RViz2 上に配置。右クリックメニューでスケール変更・形状切替が可能
- **ROS 2 パラメータ** で経路計画手法やAPFパラメータをランタイム切替
- タイマー（10Hz）で毎フレーム `initPotentialField` → `setRobot/setGoal` → `setObstacle` → `createPotentialField` → `createPath` を実行
- RViz2 の `2D Pose Estimate` でロボット位置、`Nav2 Goal` でゴール、`Publish Point` で点障害物を設定
- 経路を `planned_path` トピック、ポテンシャル場を `apf/field/potential` トピックに publish

障害物マーカーは `toPointVec()` で頂点列に変換される（CUBE は4頂点の回転変換、SPHERE は楕円の離散化）。

### Interactive Marker デモ (`interactive_marker_node`)

`src/interactive_marker_node.cpp` — メニュー付き Interactive Marker のサンプル実装。メインの経路計画機能とは独立。

### 名前空間

launch ファイルで `/tutorial` 名前空間内に起動される。トピック名は `/tutorial/initialpose`, `/tutorial/goal_pose` 等になる。

### ROS 2 パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `planning_method` | string | "dijkstra" | 経路計画手法 (dijkstra/astar) |
| `field_resolution` | double | 0.05 | ポテンシャル場の解像度 |
| `field_rows` / `field_cols` | int | 240 | ポテンシャル場のグリッドサイズ |
| `weight_attraction` | double | 0.1 | 引力場の重み |
| `weight_repulsion` | double | 0.1 | 斥力場の重み |
| `distance_threshold_repulsion` | double | 0.3 | 斥力の影響距離閾値 |
| `max_path_length` | double | 6.0 | 最大経路長 |

## 依存関係

```
potbot_lib（APF アルゴリズム、ユーティリティ関数）
potbot_ros（ROS 2 ユーティリティ：get_point, get_pose, get_quat, to_msg, field_to_pcl2）
interactive_markers, tf2, tf2_ros
```
