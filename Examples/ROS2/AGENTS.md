# Examples/ROS2/ — ROS2集成

## 概述
RGB-D RealSense D435i的ROS2 Humble节点,支持稠密点云和占用栅格地图输出。

## 文件
| 文件 | 用途 |
|------|------|
| `rgbd_realsense_D435i_ros2.cc` | 主ROS2节点 (~500行) |
| `OccupancyGridUtils.h` | 3D→2D投影 + PGM/YAML读写 |

## 坐标变换 (关键)
ORB-SLAM3相机 → ROS2 REP-103标准:
```cpp
R_CAM_TO_ROS = [[0,0,1], [-1,0,0], [0,-1,0]]  // X=右→-Y, Y=下→-Z, Z=前→X
```
所有ROS2数据出口必须应用此变换:
- PointCloud2坐标
- TF `map→odom` + 静态 `map→slam_origin`

## ROS2话题
| 话题 | 类型 | 坐标系 | 频率 |
|------|------|--------|------|
| `/dense_pointcloud` | sensor_msgs/PointCloud2 | map | 每关键帧 |
| `/tf` | tf2_msgs/TFMessage | - | 30Hz |

## TF树
```
map → slam_origin (静态) → odom (动态)
```
- `map→slam_origin`: 静态桥接 (相机→ROS坐标)
- `map→odom`: 动态SLAM位姿

## 构建
```bash
./build_ros.sh  # 或 lcompile.sh (使用本地3rdlibs)
```

## 反模式
- **禁止** 修改 PointCloudFusion.h 或核心库
- **禁止** 使用原始 `Tcw.translation()` 做TF — 必须先应用变换
- **禁止** 提交到 `build_ros/` 目录
