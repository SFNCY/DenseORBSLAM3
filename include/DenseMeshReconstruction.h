/**
 * @file DenseMeshReconstruction.h
 * @brief Interface for dense mesh reconstruction using TSDF fusion
 *
 * Uses PIMPL idiom to isolate Open3D headers from the rest of ORB-SLAM3,
 * avoiding GL/glew.h conflicts with Pangolin.
 */
#ifndef DENSE_MESH_RECONSTRUCTION_H
#define DENSE_MESH_RECONSTRUCTION_H

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <string>
#include <chrono>
#include <shared_mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <memory>
#include <fstream>

#include "PointCloudFusion.h"  // CameraIntrinsics full definition

namespace ORB_SLAM3 {

// ============================================================================
// Configuration and statistics structs
// ============================================================================

struct MeshReconConfig {
    float voxel_length = 0.005f;
    float sdf_trunc = 0.02f;
    int update_every_n_kf = 3;
    float thDepth = 3.0f;

    MeshReconConfig() = default;
    MeshReconConfig(float _voxel_length, float _sdf_trunc,
                    int _update_every_n_kf, float _thDepth)
        : voxel_length(_voxel_length), sdf_trunc(_sdf_trunc),
          update_every_n_kf(_update_every_n_kf), thDepth(_thDepth) {}
};

struct MeshStats {
    int total_integrations = 0;
    int total_extractions = 0;
    int current_voxels = 0;
    double last_extract_time_ms = 0.0;
    MeshStats() = default;
};

struct MeshTimingStats {
    std::vector<double> integrate_times;
    std::vector<double> extract_times;
    double last_integrate_ms = 0.0;
    double last_extract_ms = 0.0;
    double avg_integrate_ms = 0.0;
    double p99_integrate_ms = 0.0;

    MeshTimingStats() = default;

    void CalculateStats() {
        if (!integrate_times.empty()) {
            avg_integrate_ms = 0.0;
            for (double t : integrate_times) avg_integrate_ms += t;
            avg_integrate_ms /= integrate_times.size();

            if (integrate_times.size() >= 2) {
                size_t p99_idx = static_cast<size_t>(
                    std::floor(integrate_times.size() * 0.99));
                std::nth_element(integrate_times.begin(),
                                 integrate_times.begin() + p99_idx,
                                 integrate_times.end());
                p99_integrate_ms = integrate_times[p99_idx];
            } else if (integrate_times.size() == 1) {
                p99_integrate_ms = integrate_times[0];
            }
        }
    }
};

// ============================================================================
// DenseMeshReconstruction class
// ============================================================================

class DenseMeshReconstruction {
public:
    explicit DenseMeshReconstruction(
        const MeshReconConfig& config = MeshReconConfig());
    ~DenseMeshReconstruction();

    bool IntegrateRGBD(const cv::Mat& rgb,
                       const cv::Mat& depth,
                       const Sophus::SE3f& Tcw,
                       const CameraIntrinsics& intrinsics);

    bool ExtractMesh(bool incremental_only = false);

    bool SaveMeshPLY(const std::string& filename);
    bool SaveMeshOBJ(const std::string& filename);

    void Clear();
    MeshStats GetStats() const;
    void Reset();
    void ResetIntegrationCounter();

    std::string GetTimingReport();
    MeshTimingStats GetTimingStats();
    void OutputTimingSummary();

    int GetVertexCount();
    int GetTriangleCount();

    void GetMeshData(std::vector<Eigen::Vector3d>& vertices,
                     std::vector<Eigen::Vector3i>& triangles,
                     std::vector<Eigen::Vector3d>& colors) const;

    // Non-copyable
    DenseMeshReconstruction(const DenseMeshReconstruction&) = delete;
    DenseMeshReconstruction& operator=(const DenseMeshReconstruction&) = delete;

    // Movable
    DenseMeshReconstruction(DenseMeshReconstruction&& other) noexcept;
    DenseMeshReconstruction& operator=(DenseMeshReconstruction&& other) noexcept;

private:
#ifdef DENSE_MESH_ENABLED
    struct Impl;
    std::unique_ptr<Impl> impl_;  // PIMPL hides all Open3D types
#else
    void* stub_tsdf_volume_ = nullptr;
    void* stub_mesh_ = nullptr;
    MeshReconConfig config_;
    MeshStats stats_;
    int stub_frame_count_ = 0;
    int stub_last_extract_frame_ = 0;
#endif
};

// ============================================================================
// Stub implementations (no Open3D dependency)
// ============================================================================

#ifndef DENSE_MESH_ENABLED

inline DenseMeshReconstruction::DenseMeshReconstruction(const MeshReconConfig& config)
    : stub_tsdf_volume_(nullptr), stub_mesh_(nullptr),
      config_(config), stats_(), stub_frame_count_(0), stub_last_extract_frame_(0) {}

inline DenseMeshReconstruction::~DenseMeshReconstruction() { Clear(); }

inline bool DenseMeshReconstruction::IntegrateRGBD(
    const cv::Mat&, const cv::Mat&, const Sophus::SE3f&, const CameraIntrinsics&) {
    return false;
}

inline bool DenseMeshReconstruction::ExtractMesh(bool) { return false; }
inline bool DenseMeshReconstruction::SaveMeshPLY(const std::string&) { return false; }
inline bool DenseMeshReconstruction::SaveMeshOBJ(const std::string&) { return false; }

inline void DenseMeshReconstruction::Clear() {
    stub_tsdf_volume_ = nullptr;
    stub_mesh_ = nullptr;
    stub_frame_count_ = 0;
    stub_last_extract_frame_ = 0;
}

inline MeshStats DenseMeshReconstruction::GetStats() const {
    MeshStats result = stats_;
    result.current_voxels = 0;
    return result;
}

inline void DenseMeshReconstruction::Reset() {
    stats_ = MeshStats();
    stub_frame_count_ = 0;
    stub_last_extract_frame_ = 0;
}

inline void DenseMeshReconstruction::ResetIntegrationCounter() {
    stats_.total_integrations = 0;
}

inline std::string DenseMeshReconstruction::GetTimingReport() {
    return "DenseMesh Timing: (stub mode)";
}

inline MeshTimingStats DenseMeshReconstruction::GetTimingStats() {
    return MeshTimingStats();
}

inline void DenseMeshReconstruction::OutputTimingSummary() {}

inline int DenseMeshReconstruction::GetVertexCount() { return 0; }
inline int DenseMeshReconstruction::GetTriangleCount() { return 0; }

inline void DenseMeshReconstruction::GetMeshData(std::vector<Eigen::Vector3d>& vertices,
                                                  std::vector<Eigen::Vector3i>& triangles,
                                                  std::vector<Eigen::Vector3d>& colors) const {
    vertices.clear(); triangles.clear(); colors.clear();
}

inline DenseMeshReconstruction::DenseMeshReconstruction(
    DenseMeshReconstruction&& other) noexcept
    : stub_tsdf_volume_(other.stub_tsdf_volume_),
      stub_mesh_(other.stub_mesh_),
      config_(other.config_),
      stats_(other.stats_),
      stub_frame_count_(other.stub_frame_count_),
      stub_last_extract_frame_(other.stub_last_extract_frame_) {
    other.stub_tsdf_volume_ = nullptr;
    other.stub_mesh_ = nullptr;
    other.stub_frame_count_ = 0;
    other.stub_last_extract_frame_ = 0;
}

inline DenseMeshReconstruction& DenseMeshReconstruction::operator=(
    DenseMeshReconstruction&& other) noexcept {
    if (this != &other) {
        stub_tsdf_volume_ = other.stub_tsdf_volume_;
        stub_mesh_ = other.stub_mesh_;
        config_ = other.config_;
        stats_ = other.stats_;
        stub_frame_count_ = other.stub_frame_count_;
        stub_last_extract_frame_ = other.stub_last_extract_frame_;
        other.stub_tsdf_volume_ = nullptr;
        other.stub_mesh_ = nullptr;
        other.stub_frame_count_ = 0;
        other.stub_last_extract_frame_ = 0;
    }
    return *this;
}

#endif  // !DENSE_MESH_ENABLED

}  // namespace ORB_SLAM3

#endif  // DENSE_MESH_RECONSTRUCTION_H
