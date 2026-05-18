# 项目知识库

**Generated:** 2026-05-18 04:57
**Commit:** da1a5a6 (ros2_branch)
**最后重构:** OccupancyGridUtils.h 坐标语义修复, TF坐标变换

## 概述
DenseORBSLAM3 — ORB-SLAM3扩展版本,支持稠密网格重建和ROS2集成。C++17 SLAM库,支持单目/双目/RGB-D相机的视觉/视觉惯性/多地图SLAM。

## 目录结构
```
DenseORBSLAM3/
├── src/                 # 核心实现 (27个.cc文件)
├── include/             # 头文件 (31个.h文件)
├── Examples/            # 可执行示例 (ROS2, RGB-D, 双目, 单目)
│   └── ROS2/            # ROS2节点 (ENABLE_ROS2)
│   └── RGB-D/           # RGB-D示例 + PointCloudFusion.h
├── Thirdparty/          # DBoW2, g2o, Sophus (BSD许可)
├── Vocabulary/          # ORB词汇表 (tarball)
├── test/                # Google Test测试套件 (BUILD_TESTS=ON)
└── CMakeLists.txt       # 构建配置 (DENSE_MESH, ENABLE_ROS2, USE_DENSE_CLOUD_FALLBACK)
```

## 关键文件位置
| 任务 | 位置 | 备注 |
|------|------|------|
| 核心SLAM | `src/System.cc`, `src/Tracking.cc` | 主入口 + 跟踪 |
| 地图管理 | `src/Atlas.cc`, `src/Map.cc` | 多地图系统 |
| 稠密网格 | `src/DenseMeshReconstruction.cc` | 需要 DENSE_MESH=ON |
| ROS2节点 | `Examples/ROS2/rgbd_realsense_D435i_ros2.cc` | TF + PointCloud2 |
| 占用栅格 | `Examples/ROS2/OccupancyGridUtils.h` | 3D→2D投影 |
| 点云融合 | `Examples/RGB-D/PointCloudFusion.h` | 稠密重建 |
| 相机模型 | `src/CameraModels/` | 针孔, KannalaBrandt8 |

## 关键类映射
| 符号 | 类型 | 位置 | 职责 |
|------|------|------|------|
| System | class | include/System.h | 主SLAM接口 |
| Tracking | class | include/Tracking.h | 视觉/视觉惯性跟踪 |
| Atlas | class | include/Atlas.h | 多地图管理 |
| MapPoint | class | include/MapPoint.h | 3D点路标 |
| KeyFrame | class | include/KeyFrame.h | 关键帧(含位姿/图) |
| Optimizer | class | include/Optimizer.h | 基于g2o的优化 |
| DenseMeshReconstruction | class | include/DenseMeshReconstruction.h | TSDF融合 + 网格 |

## 约定 (本项目)
- **C++标准**: 自动检测 C++17/14/11 (CMakeLists.txt:21-39)
- **构建选项**: `DENSE_MESH`(默认OFF), `ENABLE_ROS2`(默认OFF), `USE_DENSE_CLOUD_FALLBACK`(默认ON)
- **坐标系**: ORB-SLAM3 Y=上, Z=前 (不是ROS的Z=上)
- **ROS2坐标系**: "map"为世界坐标系, "odom"为SLAM位姿
- **线程模型**: 主线程 + 跟踪 + 局部建图 + 闭环 + 可视化
- **IMU支持**: 支持单目惯性/双目惯性模式

## 反模式 (本项目禁止)
- **禁止** 直接修改 Thirdparty/g2o — 外部库
- **禁止** 直接使用 `Tcw.translation()` 做ROS2 TF — 必须应用相机→ROS坐标变换
- **禁止** 提交构建目录 (`build/`, `build_ros/` 等)
- **禁止** 在生产环境设置 `BUILD_TESTS=ON` — 仅用于测试框架

## 独有风格
- **稠密点云**: `std::vector<DensePoint>` (pos + rgb) 每关键帧累积
- **占用栅格**: 投影到ROS XY平面(不是相机XZ)
- **PLY导出**: 通过 `WritePLY()` 导出带RGB颜色的二进制PLY
- **体素降采样**: `VoxelGridDownsample()` 控制密度

## 命令
```bash
# 标准构建 (无ROS2, 无稠密网格)
./build.sh

# ROS2构建 (ENABLE_ROS2=ON)
./build_ros.sh

# 稠密网格构建
./compile.sh

# 测试构建
cmake -DBUILD_TESTS=ON ..
make -j4
ctest --output-on-failure

# 运行RGB-D示例
./Examples/build/rgbd_realsense_D435i Vocabulary/ORBvoc.txt Examples/RGB-D/RealSense_D435i.yaml

# 运行ROS2示例
./Examples/build/rgbd_realsense_D435i_ros2 Vocabulary/ORBvoc.txt Examples/RGB-D/RealSense_D435i.yaml
```

## 注意事项
- ROS2节点在 `lcompile.sh` 中硬编码路径: `/home/yucheng/3rdlib/`, `/workspace/3rdlibs/`
- `cmake_modules/` 目录缺失但被CMakeLists.txt引用
- g2o `base_binary_edge.h` 有 `// HACK` 指针映射技巧注释
- `KeyFrameDatabase.h:70` 有 `// Loop Detection(DEPRECATED)` 注释
- ~60个TODO/FIXME散布在代码库中 (LoopClosing.cc, Optimizer.cc最多)
