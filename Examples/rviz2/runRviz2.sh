#!/bin/bash

# 启动rviz2并加载DenseORBSLAM3的配置文件

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/dense_orbslam3.rviz"

# 检查配置文件是否存在
if [ ! -f "${CONFIG_FILE}" ]; then
    echo "错误：配置文件 ${CONFIG_FILE} 不存在"
    exit 1
fi

# 检查rviz2是否安装
if ! command -v rviz2 &> /dev/null; then
    echo "错误：rviz2 未安装。请先安装ROS2桌面版："
    echo "  sudo apt install ros-${ROS_DISTRO}-rviz2"
    exit 1
fi

# 设置ROS2环境（如果尚未设置）
if [ -z "${ROS_DISTRO}" ]; then
    # 尝试自动检测ROS2环境
    for distro in humble iron jazzy rolling; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            source "/opt/ros/${distro}/setup.bash"
            break
        fi
    done
fi

# 检查ROS2环境是否已设置
if [ -z "${ROS_DISTRO}" ]; then
    echo "警告：无法检测ROS2环境。请确保已source ROS2 setup.bash"
    echo "例如：source /opt/ros/humble/setup.bash"
fi

echo "启动rviz2并加载配置文件：${CONFIG_FILE}"
echo "请确保DenseORBSLAM3 ROS2节点正在运行"
echo ""

# 启动rviz2
rviz2 -d "${CONFIG_FILE}"