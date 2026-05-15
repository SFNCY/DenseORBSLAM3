/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>
#include "PointCloudFusion.h"

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "OccupancyGridUtils.h"

// Helper function to convert GlobalCloud to PointCloud2 message
sensor_msgs::msg::PointCloud2 GlobalCloudToPointCloud2(
    const std::vector<ORB_SLAM3::DensePoint>& cloud,
    const std::string& frame_id,
    rclcpp::Time timestamp,
    size_t stride = 1)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = timestamp;
    msg.height = 1;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.point_step = 16;  // 4 floats (x,y,z) + 1 uint32 (rgb) = 16 bytes

    // FIX: Use ceiling division to ensure buffer is large enough
    // for all stride-grouped elements, preventing heap overflow.
    size_t numPoints = (cloud.size() + stride - 1) / stride;
    msg.width = numPoints;
    msg.row_step = msg.point_step * msg.width;

    // Define fields: x, y, z as float32, rgb as uint32
    sensor_msgs::msg::PointField field_x, field_y, field_z, field_rgb;
    field_x.name = "x"; field_x.offset = 0;  field_x.datatype = 7; field_x.count = 1;  // 7 = FLOAT32
    field_y.name = "y"; field_y.offset = 4;  field_y.datatype = 7; field_y.count = 1;
    field_z.name = "z"; field_z.offset = 8;  field_z.datatype = 7; field_z.count = 1;
    field_rgb.name = "rgb"; field_rgb.offset = 12; field_rgb.datatype = 6; field_rgb.count = 1;  // 6 = UINT32
    msg.fields = {field_x, field_y, field_z, field_rgb};

    // Handle empty cloud
    if (cloud.empty()) {
        msg.data.resize(0);
        return msg;
    }

    msg.data.resize(msg.row_step);
    char* dataPtr = reinterpret_cast<char*>(msg.data.data());

    // FIX: Safe loop - use separate counter 'pointIdx' for offset,
    // bound by both cloud.size() and numPoints.
    for (size_t i = 0, pointIdx = 0;
         i < cloud.size() && pointIdx < numPoints;
         i += stride, pointIdx++) {
        const auto& pt = cloud[i];
        float x = pt.pos.x();
        float y = pt.pos.y();
        float z = pt.pos.z();
        // Pack RGB into uint32 (r, g, b)
        uint32_t rgb = (static_cast<uint32_t>(pt.r) << 16) |
                      (static_cast<uint32_t>(pt.g) << 8) |
                      static_cast<uint32_t>(pt.b);

        size_t offset = pointIdx * msg.point_step;
        memcpy(dataPtr + offset, &x, 4);
        memcpy(dataPtr + offset + 4, &y, 4);
        memcpy(dataPtr + offset + 8, &z, 4);
        memcpy(dataPtr + offset + 12, &rgb, 4);
    }

    return msg;
}

#endif

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./rgbd_realsense_D435i_ros2 path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Explicitly copy argv to std::string - implicit const char*→string
    // temporaries can be corrupted by library static initializers.
    std::string vocFile(argv[1]);
    std::string settingsFile(argv[2]);

    // Create SLAM system before RealSense initialization to avoid
    // potential conflicts with librealsense static constructors.
    ORB_SLAM3::System SLAM(vocFile, settingsFile, ORB_SLAM3::System::RGBD, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

#ifdef DENSE_MESH_ENABLED
    SLAM.EnableDenseMesh();
#endif

#ifdef ENABLE_ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgbd_realsense_D435i_ros2");
    RCLCPP_INFO(node->get_logger(), "ROS2 node initialized");

    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/dense_pointcloud", 10);
    RCLCPP_INFO(node->get_logger(), "PointCloud2 publisher created on /dense_pointcloud");

    auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
#endif

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 50000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
            }
            // std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            get_sensor_option(sensor);
            if (index == 2) {
                // RGB camera
                sensor.set_option(RS2_OPTION_EXPOSURE, 80.f);
            }
        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    // Depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // IMU streams are intentionally NOT enabled - this version is RGB-D only

    // Start pipeline with synchronous frame capture (no IMU callback)
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);

    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    int width_img = intrinsics_cam.width;
    int height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
    intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    // Align depth and RGB frames
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());
    rs2::align align(align_to);

    double timestamp;
    cv::Mat im, depth;

    double t_resize = 0.f;
    double t_track = 0.f;

    // Dense point cloud fusion
#if USE_DENSE_CLOUD_FALLBACK
    ORB_SLAM3::CameraIntrinsics camIntrinsics(
        intrinsics_cam.fx, intrinsics_cam.fy,
        intrinsics_cam.ppx, intrinsics_cam.ppy,
        intrinsics_cam.width, intrinsics_cam.height);
    std::vector<ORB_SLAM3::DensePoint> globalCloud;
    unsigned long lastKFCount = 0;
    int totalKFUsed = 0;
#endif

    while (!SLAM.isShutDown() && b_continue_session)
    {
        // Synchronously wait for frames (no IMU callback needed)
        rs2::frameset fs;
        try {
            fs = pipe.wait_for_frames(5000); // 5s timeout
        } catch (const rs2::error& e) {
            std::cerr << "RealSense error: " << e.what() << std::endl;
            continue;
        }

        // Check for profile changes
        if (profile_changed(pipe.get_active_profile().get_streams(), pipe_profile.get_streams()))
        {
            pipe_profile = pipe.get_active_profile();
            align_to = find_stream_to_align(pipe_profile.get_streams());
            align = rs2::align(align_to);
        }

        // Align depth to color
        auto processed = align.process(fs);

        rs2::video_frame color_frame = processed.first(align_to);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        if (!color_frame || !depth_frame) {
            continue;
        }

        timestamp = fs.get_timestamp() * 1e-3;

        // Deep copy to avoid dangling pointers to RealSense frame data
        cv::Mat(cv::Size(width_img, height_img), CV_8UC3,
                 (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP).copyTo(im);
        cv::Mat(cv::Size(width_img, height_img), CV_16U,
                 (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP).copyTo(depth);

        if (imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
        // Pass the image to the SLAM system
        Sophus::SE3f Tcw = SLAM.TrackRGBD(im, depth, timestamp);

#ifdef DENSE_MESH_ENABLED
        SLAM.PushRGBDFrame(im, depth, timestamp);
#endif

#ifdef ENABLE_ROS2
        if (SLAM.GetTrackingState() == 2) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.frame_id = "map";
            tf.child_frame_id = "odom";
            tf.header.stamp = node->now();

            Eigen::Vector3f camPos = Tcw.translation();
            tf.transform.translation.x = camPos.x();
            tf.transform.translation.y = camPos.y();
            tf.transform.translation.z = camPos.z();

            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Quaternionf q(Twc.rotationMatrix());
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            tf_broadcaster->sendTransform(tf);
        }
#endif

        // === Dense Point Cloud Fusion ===
#if USE_DENSE_CLOUD_FALLBACK
        if (SLAM.GetTrackingState() == 2) {  // Tracking OK
            unsigned long kfCount = SLAM.GetKeyFramesInMap();
            if (kfCount > lastKFCount) {
                lastKFCount = kfCount;
                totalKFUsed++;

                // Convert depth from CV_16U (mm) to CV_32F (meters)
                cv::Mat depth32F;
                depth.convertTo(depth32F, CV_32F, 1.0f / 1000.0f);

                // Extract dense points for this keyframe
                // depthFactor=1.0 because depth32F is already in meters
                std::vector<ORB_SLAM3::DensePoint> framePoints =
                    ORB_SLAM3::DepthToWorldPoints(im, depth32F, Tcw, camIntrinsics, 4, 1.0f, 40.0f);

                globalCloud.insert(globalCloud.end(), framePoints.begin(), framePoints.end());

                {
                    size_t displayStride = std::max(size_t(1), globalCloud.size() / 100000);
                    std::vector<Eigen::Vector3f> vDisplayPoints;
                    std::vector<Eigen::Matrix<unsigned char,3,1>> vDisplayColors;
                    if (displayStride > 0 && globalCloud.size() > displayStride) {
                        vDisplayPoints.reserve(globalCloud.size() / displayStride + 1);
                        vDisplayColors.reserve(globalCloud.size() / displayStride + 1);
                    }
                    for (size_t i = 0; i < globalCloud.size(); i += displayStride) {
                        const auto &pt = globalCloud[i];
                        vDisplayPoints.push_back(pt.pos);
                        vDisplayColors.push_back(Eigen::Matrix<unsigned char,3,1>(pt.r, pt.g, pt.b));
                    }
                    SLAM.SetDenseCloud(vDisplayPoints, vDisplayColors);

#ifdef ENABLE_ROS2
                    if (!globalCloud.empty()) {
                        size_t pubStride = std::max(size_t(1), globalCloud.size() / 100000);
                        auto cloudMsg = GlobalCloudToPointCloud2(
                            globalCloud, "map", node->now(), pubStride);
                        cloud_pub->publish(cloudMsg);
                    }
#endif
                }

                // Debug: show camera position and first few world points
                Eigen::Vector3f camPos = Tcw.translation();
                std::cout << "[DenseFusion] KF #" << totalKFUsed
                          << " (total KFs: " << kfCount << ")"
                          << " | Camera at (" << camPos.x() << ", " << camPos.y() << ", " << camPos.z() << ")"
                          << " | Frame pts: " << framePoints.size()
                          << " | Total pts: " << globalCloud.size() << std::endl;
            }
        }
#endif

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }
    cout << "System shutdown!" << endl;

    // Stop pipeline before saving
    pipe.stop();

    // === Save Dense Point Cloud ===
#if USE_DENSE_CLOUD_FALLBACK
    if (!globalCloud.empty()) {
        std::cout << "Downsampling point cloud from " << globalCloud.size() << " points..." << std::endl;

        std::vector<ORB_SLAM3::DensePoint> cloudDownsampled = ORB_SLAM3::VoxelGridDownsample(globalCloud, 0.01f);
        std::cout << "Downsampled to " << cloudDownsampled.size() << " points." << std::endl;
        if (ORB_SLAM3::WritePLY("dense_cloud.ply", cloudDownsampled)) {
            std::cout << "Saved: dense_cloud.ply" << std::endl;
        } else {
            std::cerr << "Failed to save dense_cloud.ply!" << std::endl;
        }

#ifdef ENABLE_ROS2
        std::vector<ORB_SLAM3::DensePoint> gridDownsampled = ORB_SLAM3::VoxelGridDownsample(globalCloud, 0.02f);

        std::vector<GridPoint3D> gridPoints;
        gridPoints.reserve(gridDownsampled.size());
        for (const auto& dp : gridDownsampled) {
            gridPoints.push_back(GridPoint3D(dp.pos.x(), dp.pos.y(), dp.pos.z()));
        }

        OccupancyGrid occGrid = ProjectPointCloudToGrid(gridPoints, 0.05f);

        if (WriteOccupancyGridPGM(occGrid, "map.pgm") &&
            WriteOccupancyGridYAML(occGrid, "map.pgm", "map.yaml")) {
            std::cout << "Saved: map.pgm + map.yaml (OccupancyGrid)" << std::endl;
        } else {
            std::cerr << "Failed to save OccupancyGrid files!" << std::endl;
        }
#endif

    } else {
        std::cout << "No points collected for output files." << std::endl;
    }
#endif

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    std::cout << "Saved: KeyFrameTrajectory.txt" << std::endl;

#ifdef ENABLE_ROS2
    rclcpp::shutdown();
#endif

    return 0;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}