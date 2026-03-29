# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

ROS 1 (Noetic) ベースの Artificial Potential Field (APF) 経路計画チュートリアルパッケージ。RViz 上の Interactive Markers で障害物を配置・操作しながら、APF による経路計画をリアルタイムに確認できる教育用ツール。

## ビルド・起動

```bash
# ビルド（catkin ワークスペースのルートで実行）
cd ~/catkin_ws && catkin build apf_pathplanner_tutorial

# 起動（RViz + 経路計画ノード）
roslaunch apf_pathplanner_tutorial apf_pathplanner_tutorial.launch
```

依存パッケージ `potbot_core`（potbot_base, potbot_lib, potbot_msgs）が `~/catkin_ws/src` に必要。

## アーキテクチャ

### メインノード (`apf_pathplanner_tutorial`)

`src/apf_pathplanner_tutorial.cpp` — `MarkerPathPlanner` クラスがすべてを統括：

- **pluginlib** で `potbot_nav/APF`（`potbot_base::PathPlanner` 派生）を動的にロードし経路計画を実行
- **Interactive Markers** で障害物（CUBE/SPHERE）を RViz 上に配置。右クリックメニューでスケール変更・形状切替が可能
- **dynamic_reconfigure** で経路計画手法（`weight` / `edge`）をランタイム切替（`cfg/APFPathPlannerTutorial.cfg`）
- メインループで毎フレーム `setRobot` → `setTargetPose` → `setObstacles` → `planPath` を実行
- RViz の `2D Pose Estimate` でロボット位置、`2D Nav Goal` でゴール、`Publish Point` で点障害物を設定

障害物マーカーは `toPointVec()` で頂点列に変換される（CUBE は4頂点の回転変換、SPHERE は楕円の離散化）。

### Interactive Marker デモ (`interactive_marker_node`)

`src/interactive_marker_node.cpp` — メニュー付き Interactive Marker のサンプル実装（Willow Garage 由来）。メインの経路計画機能とは独立。

### 名前空間

launch ファイルで `/tutorial` 名前空間内に起動される。トピック名は `/tutorial/initialpose`, `/tutorial/move_base_simple/goal` 等になる。

## 依存関係

```
potbot_base（プラグインインターフェース）
potbot_lib（APF アルゴリズム、ユーティリティ関数）
potbot_msgs（カスタムメッセージ）
pluginlib, pcl_ros, dynamic_reconfigure, interactive_markers
```
