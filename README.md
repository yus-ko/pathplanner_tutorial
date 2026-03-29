# apf_pathplanner_tutorial

# ビルド方法

```bash
cd ~/ros2_ws/src
git clone https://github.com/yus-ko/potbot_core
git clone https://github.com/yus-ko/apf_pathplanner_tutorial
```
```bash
cd ~/ros2_ws
colcon build --packages-select apf_pathplanner_tutorial
```

# 起動方法

```bash
ros2 launch apf_pathplanner_tutorial apf_pathplanner_tutorial.launch.py
```
