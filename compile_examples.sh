clear
set -e

# =============================================================================
# mono_euroc Usage Instructions
# =============================================================================
# mono_euroc runs ORB-SLAM3 in monocular mode on the EuRoC MAV dataset.
#
# Syntax:
#   ./mono_euroc <path_to_vocabulary> <path_to_settings> \\
#                <path_to_sequence_folder> <path_to_times_file> \\
#                [trajectory_file_name]
#
# Arguments:
#   1. path_to_vocabulary   : ORB vocabulary file (binary or text).
#                             Typically: Vocabulary/ORBvoc.txt
#   2. path_to_settings     : YAML configuration file for the sensor setup.
#                             Typically: Examples/Monocular/EuRoC.yaml
#   3. path_to_sequence     : Root directory of a EuRoC sequence.
#                             e.g., /media/yucheng/Data/Data/ORB-SLAM3/machine_hall/MH_01_easy
#   4. path_to_times_file   : Text file containing image timestamps.
#                             e.g., Examples/Monocular/EuRoC_TimeStamps/MH01.txt
#   5. trajectory_file_name : (Optional) Base name for output trajectory files.
#
# Example Command (Vicon Room 1 01 Easy):
# ------------------------------------------------------------------------------
# Set runtime library paths for custom Pangolin installation
# export LD_LIBRARY_PATH=/home/yucheng/3rdlib/Pangolin-0.9.4/install/lib:$LD_LIBRARY_PATH

# ./Examples/build/mono_euroc \
#     Vocabulary/ORBvoc.txt \
#     Examples/Monocular/EuRoC.yaml \
#     /media/yucheng/Data/Data/ORB-SLAM3/vicon_room1/V1_01_easy/V1_01_easy \
#     Examples/Monocular/EuRoC_TimeStamps/V101.txt \
#     V101_mono
# ------------------------------------------------------------------------------
#
# Available EuRoC Sequences (Dataset path: /media/yucheng/Data/Data/ORB-SLAM3/):
#   Machine Hall : machine_hall/MH_01_easy ... MH_05_difficult
#   Vicon Room 1 : vicon_room1/V1_01_easy/V1_01_easy ... V1_03_difficult/V1_03_difficult
#   Vicon Room 2 : vicon_room2/V2_01_easy/V2_01_easy ... V2_03_difficult/V2_03_difficult
#
# Corresponding timestamp files are located in Examples/Monocular/EuRoC_TimeStamps/.
# =============================================================================

# =============================================================================
# RealSense D435i Usage Instructions
# =============================================================================
# stereo_inertial_realsense_D435i runs ORB-SLAM3 in stereo-inertial mode
# using an Intel RealSense D435i camera connected via USB.
#
# Syntax:
#   ./stereo_inertial_realsense_D435i <path_to_vocabulary> <path_to_settings>
#
# Arguments:
#   1. path_to_vocabulary : ORB vocabulary file (binary or text).
#                           Typically: Vocabulary/ORBvoc.txt
#   2. path_to_settings   : YAML configuration file for the RealSense D435i.
#                           Typically: Examples/Stereo-Inertial/RealSense_D435i.yaml
#
# Example Command:
# ------------------------------------------------------------------------------
export LD_LIBRARY_PATH=/home/yucheng/3rdlib/Pangolin-0.9.4/install/lib:$LD_LIBRARY_PATH

# ./Examples/build/stereo_inertial_realsense_D435i \
#     Vocabulary/ORBvoc.txt \
#     Examples/Stereo-Inertial/RealSense_D435i.yaml

# ./Examples/build/rgbd_inertial_realsense_D435i \
#     Vocabulary/ORBvoc.txt \
#     Examples/RGB-D-Inertial/RealSense_D435i.yaml

./Examples/build/rgbd_realsense_D435i\
    Vocabulary/ORBvoc.txt \
    Examples/RGB-D/RealSense_D435i.yaml
# ------------------------------------------------------------------------------
#
# Requirements:
#   - Intel RealSense D435i camera connected via USB3
#   - librealsense2 SDK installed and detected by CMake
#   - Camera streams configured in RealSense_D435i.yaml:
#       - Infrared Left & Right (640x480 @ 30fps)
#       - Accelerometer & Gyroscope (IMU)
#
# Controls:
#   - Press Ctrl+C or close the Pangolin window to stop the session gracefully.
# =============================================================================