# src/ — 核心实现

## 概述
27个.cc文件,实现ORB-SLAM3核心SLAM系统。

## 关键文件
| 文件 | 职责 |
|------|------|
| `System.cc` | 主入口,初始化所有线程 |
| `Tracking.cc` | 视觉/视觉惯性跟踪 (TODO最多) |
| `Atlas.cc` | 多地图管理 |
| `Optimizer.cc` | 基于g2o的束调整和位姿优化 |
| `LoopClosing.cc` | 地点识别+闭环 |
| `DenseMeshReconstruction.cc` | TSDF融合+网格生成 (仅DENSE_MESH) |
| `LocalMapping.cc` | 关键帧插入+地图点剔除 |

## 约定
- **线程模型**: 主线程 + 跟踪 + 局部建图 + 闭环 + 可视化
- **坐标**: ORB-SLAM3 Y=上, Z=前
- **IMU集成**: ImuTypes.h + G2oTypes.cc 用于视觉惯性

## 反模式
- **禁止** 修改 `Thirdparty/g2o/` — 外部库
- **禁止** 直接使用 `Tcw.translation()` 做ROS2 TF,必须先做坐标变换

## 注意事项
- ~60个TODO/FIXME (LoopClosing.cc, Optimizer.cc最多)
- `KeyFrameDatabase.h:70` 有废弃的环路检测
- g2o `base_binary_edge.h` 有 `// HACK` 指针映射
