# include/ — 头文件

## 概述
31个.h文件,定义ORB-SLAM3核心系统公开API。

## 关键头文件
| 头文件 | 类 | 职责 |
|--------|----|------|
| `System.h` | System | 主SLAM接口 |
| `Tracking.h` | Tracking | 视觉/视觉惯性跟踪状态机 |
| `Atlas.h` | Atlas | 多地图容器 |
| `MapPoint.h` | MapPoint | 3D点路标 |
| `KeyFrame.h` | KeyFrame | 关键帧(含位姿+共视图) |
| `Optimizer.h` | Optimizer | g2o优化例程 |
| `DenseMeshReconstruction.h` | DenseMeshReconstruction | TSDF融合 (仅DENSE_MESH) |
| `CameraModels/` | Pinhole, KannalaBrandt8 | 相机校准模型 |
| `ImuTypes.h` | ImuCalibrator, PreintegratedIMU | IMU预处理 |

## 约定
- **C++标准**: 自动检测 C++17/14/11
- **稠密输出**: `std::vector<DensePoint>` 用于RGB-D点云
- **PLY导出**: `WritePLY()` 导出带RGB的二进制PLY

## 反模式
- **禁止** 在核心头文件中包含ROS2头文件 — 使用 `#ifdef ENABLE_ROS2`
- **禁止** 直接修改Thirdparty头文件

## 注意事项
- `Config.h` 有 `REGISTER_TIMES` 标志用于性能分析
- `SerializationUtils.h` 用于地图保存/加载
- `KeyFrameRGBDData.h` 用于RGB-D专用关键帧数据
