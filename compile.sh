clear
set -e

compiler_tool="Unix Makefiles"
# compiler_tool="Ninja"

build_config_type=Release
# build_config_type=Debug
current_abs_directory=$(dirname "$(readlink -f "$0")")
# thrid_libs_directory=${current_abs_directory}/3rdlibs

# build_directory=${current_abs_directory}/build/${build_config_type}
build_directory=${current_abs_directory}/build

install_directory=${current_abs_directory}/install

cmake -E rm -rf ${build_directory}/CMakeCache.txt
# cmake -E rm -rf ${build_directory}
# cmake -E make_directory ${build_directory}
# cmake -E rm -rf ${install_directory}
cmake -G "${compiler_tool}"                                                                \
-S ${current_abs_directory}                                                                \
-B ${build_directory}                                                                      \
-DCMAKE_BUILD_TYPE=${build_config_type}                                                    \
-DCMAKE_INSTALL_PREFIX=${install_directory}                                                 \
-DCMAKE_EXPORT_COMPILE_COMMANDS="ON"                                                      \
-DDENSE_MESH="ON"                                                                          \
-DOpen3D_DIR=/home/yucheng/3rdlib/Open3D-0.19.0/install/lib/cmake/Open3D/                \
-DEigen3_DIR=/home/yucheng/3rdlib/Eigen-3.4.0/install/lib/cmake/eigen3/                  \
-DOpenCV_DIR=/home/yucheng/3rdlib/opencv-4.10.0/install/lib/cmake/opencv4/               \
-DPangolin_DIR=/home/yucheng/3rdlib/Pangolin-0.9.4/install/lib/cmake/Pangolin/            \
-Drealsense2_DIR=/home/yucheng/yuchengGit/librealsense/install/lib/cmake/realsense2

# log_file="build_$(date +%Y%m%d_%H%M%S).log"
log_file="compile.log"
# cmake --build ${build_directory} --config ${build_config_type} --parallel 20 > ${log_file} 2>&1
cmake --build ${build_directory} --config ${build_config_type} --parallel 20
cmake --install ${build_directory} --config ${build_config_type} --prefix ${install_directory}

# rm -rf "${current_abs_directory}/compile_commands.json"
# cp "${build_directory}/compile_commands.json" "${current_abs_directory}/compile_commands.json"