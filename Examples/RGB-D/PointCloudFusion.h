/**
 * @file PointCloudFusion.h
 * @brief RGB-D point cloud fusion utilities for ORB-SLAM3
 */
#ifndef POINT_CLOUD_FUSION_H
#define POINT_CLOUD_FUSION_H

#include <vector>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <fstream>
#include <iomanip>

namespace ORB_SLAM3 {

/**
 * @brief Dense 3D point with RGB color
 */
struct DensePoint {
    Eigen::Vector3f pos;  // x, y, z position in world frame
    uint8_t r, g, b;      // RGB color channels (converted from BGR)
};

/**
 * @brief Camera intrinsics for depth backprojection
 */
struct CameraIntrinsics {
    float fx, fy, cx, cy;
    float invfx, invfy;
    int width, height;

    CameraIntrinsics() : fx(617.201f), fy(617.362f), cx(324.637f), cy(242.462f),
                         invfx(1.0f/fx), invfy(1.0f/fy), width(640), height(480) {}

    CameraIntrinsics(float _fx, float _fy, float _cx, float _cy, int _width, int _height)
        : fx(_fx), fy(_fy), cx(_cx), cy(_cy), width(_width), height(_height)
    {
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;
    }
};

/**
 * @brief Backproject depth map to world 3D points with color
 * @param imRGB      RGB image (BGR format from OpenCV)
 * @param imDepth    Depth image (raw values in meters as CV_32F)
 * @param Tcw        Camera to world transformation (SE3f), Pw = Tcw * Pc
 * @param intrinsics Camera intrinsics (fx, fy, cx, cy, invfx, invfy)
 * @param step       Sampling step for downsampling
 * @param depthFactor Depth value multiplier (depth_raw * depthFactor = real depth in meters)
 * @param thDepth    Maximum depth threshold in meters (points beyond this are filtered)
 * @return Vector of DensePoint with 3D positions and colors in world frame
 */
inline std::vector<DensePoint> DepthToWorldPoints(
    cv::Mat &imRGB,
    cv::Mat &imDepth,
    const Sophus::SE3f &Tcw,
    const CameraIntrinsics &intrinsics,
    int step,
    float depthFactor,
    float thDepth)
{
    std::vector<DensePoint> points;

    const float cx = intrinsics.cx;
    const float cy = intrinsics.cy;
    const float invfx = intrinsics.invfx;
    const float invfy = intrinsics.invfy;

    const int rows = imDepth.rows;
    const int cols = imDepth.cols;

    // Pre-compute camera-to-world transform (mTcw in ORB_SLAM3 is world→camera)
    const Sophus::SE3f Twc = Tcw.inverse();

    for (int v = 0; v < rows; v += step) {
        for (int u = 0; u < cols; u += step) {
            float z = imDepth.at<float>(v, u);

            if (z <= 0.0f || z > thDepth)
                continue;

            float x_cam = (u - cx) * z * invfx;
            float y_cam = (v - cy) * z * invfy;

            // Transform camera→world using Twc = Tcw.inverse()
            // Matches Frame::UnprojectStereo: mRwc*Pc + mOw = Twc*Pc
            Eigen::Vector3f Pw = Twc * Eigen::Vector3f(x_cam, y_cam, z);

            // Get RGB color (OpenCV uses BGR format, convert to RGB for PLY)
            cv::Vec3b bgr = imRGB.at<cv::Vec3b>(v, u);
            uint8_t b = bgr[0];
            uint8_t g = bgr[1];
            uint8_t r = bgr[2];

            DensePoint pt;
            pt.pos = Pw;
            pt.r = r;
            pt.g = g;
            pt.b = b;

            points.push_back(pt);
        }
    }

    return points;
}

/**
 * @brief Write point cloud to PLY file in ASCII format
 * @param filename Output PLY file path
 * @param points Vector of dense points to write
 * @return true if write successful, false otherwise
 */
inline bool WritePLY(const std::string &filename, const std::vector<DensePoint> &points)
{
    std::ofstream ofs(filename.c_str());
    if (!ofs.is_open())
    {
        return false;
    }

    // Write PLY header
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "comment Created by ORB_SLAM3 dense fusion\n";
    ofs << "element vertex " << points.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";

    // Write point data
    ofs << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < points.size(); ++i)
    {
        // BGR to RGB order as specified: write r, g, b
        ofs << points[i].pos.x() << " " << points[i].pos.y() << " " << points[i].pos.z() << " "
            << static_cast<unsigned int>(points[i].r) << " "
            << static_cast<unsigned int>(points[i].g) << " "
            << static_cast<unsigned int>(points[i].b) << "\n";
    }

    ofs.close();
    return true;
}

/**
 * @brief Voxel grid downsampling for point clouds
 * @param points Input point cloud (modified in place)
 * @param voxelSize Size of voxel grid cell in meters
 * @return Vector of downsampled points (centroid of each voxel)
 */
inline std::vector<DensePoint> VoxelGridDownsample(std::vector<DensePoint> &points, float voxelSize = 0.01f)
{
    std::vector<DensePoint> downsampled;

    if (points.empty()) {
        return downsampled;
    }

    std::unordered_map<long long, std::vector<size_t>> voxelBins;

    for (size_t i = 0; i < points.size(); ++i) {
        const auto &pt = points[i];
        long long ix = static_cast<long long>(std::floor(pt.pos.x() / voxelSize));
        long long iy = static_cast<long long>(std::floor(pt.pos.y() / voxelSize));
        long long iz = static_cast<long long>(std::floor(pt.pos.z() / voxelSize));

        long long key = ((ix & 0x1FFFFF) << 42) | ((iy & 0x1FFFFF) << 21) | (iz & 0x1FFFFF);

        voxelBins[key].push_back(i);
    }

    downsampled.reserve(voxelBins.size());

    for (const auto &pair : voxelBins) {
        const auto &indices = pair.second;

        float sumX = 0, sumY = 0, sumZ = 0;
        float sumR = 0, sumG = 0, sumB = 0;

        for (size_t idx : indices) {
            const auto &pt = points[idx];
            sumX += pt.pos.x();
            sumY += pt.pos.y();
            sumZ += pt.pos.z();
            sumR += pt.r;
            sumG += pt.g;
            sumB += pt.b;
        }

        size_t count = indices.size();
        DensePoint centroid;
        centroid.pos = Eigen::Vector3f(sumX / count, sumY / count, sumZ / count);
        centroid.r = static_cast<uint8_t>(sumR / count);
        centroid.g = static_cast<uint8_t>(sumG / count);
        centroid.b = static_cast<uint8_t>(sumB / count);

        downsampled.push_back(centroid);
    }

    return downsampled;
}

}  // namespace ORB_SLAM3

#endif  // POINT_CLOUD_FUSION_H
