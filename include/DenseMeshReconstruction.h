/**
 * @file DenseMeshReconstruction.h
 * @brief Header-only interface for dense mesh reconstruction using TSDF fusion
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

// Forward declaration to avoid including PointCloudFusion.h
namespace ORB_SLAM3 {
struct CameraIntrinsics;
}

#ifdef DENSE_MESH_ENABLED
#include <open3d/Open3D.h>
#endif

namespace ORB_SLAM3 {

/**
 * @brief Configuration parameters for mesh reconstruction
 */
struct MeshReconConfig {
    float voxel_length = 0.005f;      // Voxel size for TSDF voxel grid (meters)
    float sdf_trunc = 0.02f;           // Truncation distance for SDF (meters)
    int update_every_n_kf = 3;         // Integrate every N keyframes
    float thDepth = 3.0f;             // Maximum depth threshold (meters)

    MeshReconConfig() = default;

    MeshReconConfig(float _voxel_length, float _sdf_trunc, int _update_every_n_kf, float _thDepth)
        : voxel_length(_voxel_length), sdf_trunc(_sdf_trunc),
          update_every_n_kf(_update_every_n_kf), thDepth(_thDepth) {}
};

/**
 * @brief Statistics for mesh reconstruction
 */
struct MeshStats {
    int total_integrations = 0;        // Number of RGB-D integrations performed
    int total_extractions = 0;         // Number of mesh extractions performed
    int current_voxels = 0;            // Current number of active voxels
    double last_extract_time_ms = 0.0; // Time taken for last mesh extraction (milliseconds)

    MeshStats() = default;
};

/**
 * @brief Performance timing statistics for dense mesh reconstruction
 */
struct MeshTimingStats {
    std::vector<double> integrate_times;   // Individual integration times (ms)
    std::vector<double> extract_times;      // Individual extraction times (ms)
    double last_integrate_ms = 0.0;        // Last integration time (ms)
    double last_extract_ms = 0.0;          // Last extraction time (ms)
    double avg_integrate_ms = 0.0;         // Average integration time (ms)
    double p99_integrate_ms = 0.0;         // P99 integration time (ms)

    MeshTimingStats() = default;

    // Calculate average and p99 from stored times
    void CalculateStats() {
        if (!integrate_times.empty()) {
            avg_integrate_ms = 0.0;
            for (double t : integrate_times) {
                avg_integrate_ms += t;
            }
            avg_integrate_ms /= integrate_times.size();

            // Calculate p99 using nth_element
            if (integrate_times.size() >= 2) {
                size_t p99_idx = static_cast<size_t>(std::floor(integrate_times.size() * 0.99));
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

/**
 * @brief Dense mesh reconstruction class using TSDF fusion
 *
 * This class provides an interface for 3D mesh reconstruction from RGB-D frames
 * using Truncated Signed Distance Function (TSDF) fusion. It can work with or
 * without Open3D backend (stub implementations when DENSE_MESH_ENABLED is not defined).
 */
class DenseMeshReconstruction {
public:
    /**
     * @brief Construct a new DenseMeshReconstruction with given configuration
     * @param config Configuration parameters for reconstruction
     */
    explicit DenseMeshReconstruction(const MeshReconConfig& config = MeshReconConfig());

    /**
     * @brief Destructor - cleans up resources
     */
    ~DenseMeshReconstruction();

    /**
     * @brief Integrate an RGB-D frame into the TSDF volume
     * @param rgb RGB image (BGR format from OpenCV, CV_8UC3)
     * @param depth Depth image (raw values in meters as CV_32F, 0 = invalid)
     * @param Tcw Camera to world transformation (SE3f), Pw = Tcw * Pc
     * @param intrinsics Camera intrinsics (fx, fy, cx, cy, width, height)
     * @return true if integration successful, false otherwise
     */
    bool IntegrateRGBD(const cv::Mat& rgb,
                       const cv::Mat& depth,
                       const Sophus::SE3f& Tcw,
                       const CameraIntrinsics& intrinsics);

    /**
     * @brief Extract mesh from the TSDF volume
     * @param incremental_only If true, only extract incrementally updated region
     * @return true if extraction successful, false otherwise
     */
    bool ExtractMesh(bool incremental_only = false);

    /**
     * @brief Save mesh to PLY file
     * @param filename Output PLY file path
     * @return true if save successful, false otherwise
     */
    bool SaveMeshPLY(const std::string& filename);

    /**
     * @brief Save mesh to OBJ file
     * @param filename Output OBJ file path
     * @return true if save successful, false otherwise
     */
    bool SaveMeshOBJ(const std::string& filename);

    /**
     * @brief Clear all data and reset to initial state
     */
    void Clear();

    /**
     * @brief Get current reconstruction statistics
     * @return MeshStats structure with current statistics
     */
    MeshStats GetStats() const;

    /**
     * @brief Reset all statistics counters
     */
    void Reset();

    /**
     * @brief Reset only the integration counter
     */
    void ResetIntegrationCounter();

    /**
     * @brief Get formatted timing report string
     * @return String with formatted timing statistics
     */
    std::string GetTimingReport();

    /**
     * @brief Get timing statistics
     * @return MeshTimingStats with current timing data
     */
    MeshTimingStats GetTimingStats();

    /**
     * @brief Output timing summary to stdout (call on shutdown)
     */
    void OutputTimingSummary();

    /**
     * @brief Get number of vertices in current mesh
     * @return Vertex count, or 0 if no mesh available
     */
    int GetVertexCount();

    /**
     * @brief Get number of triangles in current mesh
     * @return Triangle count, or 0 if no mesh available
     */
    int GetTriangleCount();

#ifdef DENSE_MESH_ENABLED
private:
    // Open3D TSDF voxel grid for actual implementation
    std::shared_ptr<open3d::t::geometry::TSDFVoxelGrid> tsdf_volume_;

    // Open3D mesh for extracted geometry
    open3d::t::geometry::TriangleMesh open3d_mesh_;

    // Configuration
    MeshReconConfig config_;

    // Statistics
    MeshStats stats_;

#ifdef REGISTER_DENSE_TIMES
    // Timing statistics
    MeshTimingStats timing_stats_;
#endif

    // Frame counter for update_every_n_kf logic
    int frame_count_;
    int last_extract_frame_;

    // Mutex for thread safety
    std::shared_mutex mutex_;

    // Extracted mesh data (Eigen format for external use)
    std::vector<Eigen::Vector3d> vertices_;
    std::vector<Eigen::Vector3i> triangles_;
    std::vector<Eigen::Vector3d> vertex_colors_;

    // Helper method to initialize TSDF volume lazily
    void InitVolume();

    // Helper method to check if integration should happen
    bool ShouldIntegrate() const;

    // Internal integration method
    bool IntegrateFrame(const cv::Mat& rgb,
                        const cv::Mat& depth,
                        const Sophus::SE3f& Tcw,
                        const CameraIntrinsics& intrinsics);

    // Helper method to check if a depth value is valid
    bool IsDepthValid(float z) const;

#else // DENSE_MESH_ENABLED stub
private:
    // Stub private members to satisfy declaration
    void* stub_tsdf_volume_;
    void* stub_mesh_;
    MeshReconConfig config_;
    MeshStats stats_;
    int stub_frame_count_;
    int stub_last_extract_frame_;
#endif

public:
    // Disable copying to prevent issues with Open3D resources
    DenseMeshReconstruction(const DenseMeshReconstruction&) = delete;
    DenseMeshReconstruction& operator=(const DenseMeshReconstruction&) = delete;

    // Allow move operations
    DenseMeshReconstruction(DenseMeshReconstruction&& other) noexcept;
    DenseMeshReconstruction& operator=(DenseMeshReconstruction&& other) noexcept;
};

// ============================================================================
// Inline implementation (header-only)
// ============================================================================

#ifdef DENSE_MESH_ENABLED

inline DenseMeshReconstruction::DenseMeshReconstruction(const MeshReconConfig& config)
    : config_(config),
      stats_(),
      frame_count_(0),
      last_extract_frame_(0)
{
}

inline void DenseMeshReconstruction::InitVolume() {
    if (tsdf_volume_ != nullptr) {
        return;
    }
    open3d::core::Device device("CPU:0");
    open3d::core::Dtype dtype = open3d::core::Dtype::Float32;

    open3d::t::geometry::TSDFVoxelGrid::Params params;
    params.voxel_size = config_.voxel_length;
    params.sdf_trunc = config_.sdf_trunc;
    params.block_resolution = 16;
    params.block_count = 10000;

    tsdf_volume_ = std::make_shared<open3d::t::geometry::TSDFVoxelGrid>(params, dtype, device);
}

inline DenseMeshReconstruction::~DenseMeshReconstruction() {
    // Explicit destruction for Open3D shared_ptr
    tsdf_volume_.reset();
}

inline bool DenseMeshReconstruction::ShouldIntegrate() const {
    return (frame_count_ % config_.update_every_n_kf) == 0;
}

inline bool DenseMeshReconstruction::IsDepthValid(float z) const {
    return z > 0.1f && z < config_.thDepth;
}

inline bool DenseMeshReconstruction::IntegrateFrame(
    const cv::Mat& rgb,
    const cv::Mat& depth,
    const Sophus::SE3f& Tcw,
    const CameraIntrinsics& intrinsics)
{
    if (!ShouldIntegrate()) {
        return false;
    }

    cv::Mat depth_filtered = depth.clone();
    for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
            float z = depth.at<float>(v, u);
            if (!IsDepthValid(z)) {
                depth_filtered.at<float>(v, u) = 0.0f;
            }
        }
    }

    open3d::core::Device device("CPU:0");

    cv::Mat rgb_rgb;
    cv::cvtColor(rgb, rgb_rgb, cv::COLOR_BGR2RGB);
    open3d::core::Tensor rgb_tensor(
        rgb_rgb.reshape(3).clone(),
        {static_cast<int64_t>(rgb.rows), static_cast<int64_t>(rgb.cols), 3},
        open3d::core::Dtype::UInt8, device);

    open3d::core::Tensor depth_tensor(
        depth_filtered.clone(),
        {static_cast<int64_t>(depth_filtered.rows), static_cast<int64_t>(depth_filtered.cols)},
        open3d::core::Dtype::Float32, device);

    open3d::t::geometry::Image o3d_rgb(rgb_tensor);
    open3d::t::geometry::Image o3d_depth(depth_tensor);

    open3d::core::Tensor intrinsic_data = open3d::core::Tensor::Init<double>(
        {{intrinsics.fx, 0.0, intrinsics.cx},
         {0.0, intrinsics.fy, intrinsics.cy},
         {0.0, 0.0, 1.0}}, device);
    open3d::t::geometry::CameraIntrinsics o3d_intrinsics(intrinsic_data);

    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix4f Twc_matrix = Twc.matrix().cast<double>();
    open3d::core::Tensor extrinsics = open3d::core::Tensor::FromEigenMatrix(Twc_matrix, device);

    tsdf_volume_->Integrate(o3d_rgb, o3d_depth, o3d_intrinsics, extrinsics);

    stats_.total_integrations++;
    frame_count_++;

    return true;
}

inline bool DenseMeshReconstruction::IntegrateRGBD(
    const cv::Mat& rgb,
    const cv::Mat& depth,
    const Sophus::SE3f& Tcw,
    const CameraIntrinsics& intrinsics)
{
    if (rgb.type() != CV_8UC3) {
        return false;
    }
    if (depth.type() != CV_32F) {
        return false;
    }

    if (rgb.rows != depth.rows || rgb.cols != depth.cols) {
        return false;
    }

    if (intrinsics.fx <= 0 || intrinsics.fy <= 0) {
        return false;
    }

    if (rgb.rows != intrinsics.height || rgb.cols != intrinsics.width) {
        return false;
    }

    int total_pixels = rgb.rows * rgb.cols;
    int invalid_pixels = 0;
    for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
            float z = depth.at<float>(v, u);
            if (!IsDepthValid(z)) {
                invalid_pixels++;
            }
        }
    }

    float invalid_ratio = static_cast<float>(invalid_pixels) / static_cast<float>(total_pixels);
    if (invalid_ratio > 0.90f) {
        return false;
    }

    std::unique_lock<std::shared_mutex> lock(mutex_);
    InitVolume();

#ifdef REGISTER_DENSE_TIMES
    auto integrate_start = std::chrono::steady_clock::now();
#endif

    bool result = IntegrateFrame(rgb, depth, Tcw, intrinsics);

#ifdef REGISTER_DENSE_TIMES
    auto integrate_end = std::chrono::steady_clock::now();
    auto integrate_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        integrate_end - integrate_start);
    double integrate_ms = static_cast<double>(integrate_duration.count());

    timing_stats_.last_integrate_ms = integrate_ms;
    timing_stats_.integrate_times.push_back(integrate_ms);
    timing_stats_.CalculateStats();

    if (integrate_ms > 100.0) {
        std::cout << "DenseMesh WARNING: IntegrateRGBD took " << integrate_ms
                  << "ms (>100ms threshold)" << std::endl;
    }
#endif

    return result;
}

inline bool DenseMeshReconstruction::ExtractMesh(bool incremental_only) {
    std::shared_lock<std::shared_mutex> lock(mutex_);

#ifdef REGISTER_DENSE_TIMES
    auto start_time = std::chrono::steady_clock::now();
#endif

    if (tsdf_volume_->IsEmpty()) {
        return false;
    }

    open3d_mesh_ = tsdf_volume_->ExtractTriangleMesh();

    if (open3d_mesh_.IsEmpty()) {
        return false;
    }

    auto vertex_positions = open3d_mesh_.GetVertexPositions();
    auto triangle_indices = open3d_mesh_.GetTriangleIndices();
    auto vertex_colors = open3d_mesh_.GetVertexColors();

    int num_vertices = vertex_positions.GetShape(0);
    int num_triangles = triangle_indices.GetShape(0);

    vertices_.resize(num_vertices);
    triangles_.resize(num_triangles);
    vertex_colors_.resize(num_vertices);

    Eigen::MatrixXd vert_matrix = vertex_positions.ToEigenMatrix();
    for (int i = 0; i < num_vertices; ++i) {
        vertices_[i] = vert_matrix.row(i).cast<double>();
    }

    Eigen::MatrixXi tri_matrix = triangle_indices.ToEigenMatrix();
    for (int i = 0; i < num_triangles; ++i) {
        triangles_[i] = tri_matrix.row(i).cast<int>();
    }

    if (vertex_colors.GetShape(0) > 0) {
        Eigen::MatrixXd color_matrix = vertex_colors.ToEigenMatrix();
        for (int i = 0; i < num_vertices; ++i) {
            vertex_colors_[i] = color_matrix.row(i).cast<double>() / 255.0;
        }
    }

#ifdef REGISTER_DENSE_TIMES
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    double extract_ms = static_cast<double>(duration.count());
    timing_stats_.last_extract_ms = extract_ms;
    timing_stats_.extract_times.push_back(extract_ms);
    stats_.last_extract_time_ms = extract_ms;
#else
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    stats_.last_extract_time_ms = static_cast<double>(duration.count());
#endif

    stats_.current_voxels = tsdf_volume_->GetVoxelCount();
    stats_.total_extractions++;

    return true;
}

inline bool DenseMeshReconstruction::SaveMeshPLY(const std::string& filename) {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    if (open3d_mesh_.IsEmpty()) {
        return false;
    }

    std::ofstream test_file(filename);
    if (!test_file.is_open()) {
        return false;
    }
    test_file.close();

    auto legacy_mesh = open3d_mesh_.ToLegacyTriangleMesh();
    return open3d::io::WriteTriangleMesh(filename, legacy_mesh);
}

inline bool DenseMeshReconstruction::SaveMeshOBJ(const std::string& filename) {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    if (open3d_mesh_.IsEmpty()) {
        return false;
    }

    std::ofstream test_file(filename);
    if (!test_file.is_open()) {
        return false;
    }
    test_file.close();

    auto legacy_mesh = open3d_mesh_.ToLegacyTriangleMesh();
    return open3d::io::WriteTriangleMesh(filename, legacy_mesh);
}

inline void DenseMeshReconstruction::Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    tsdf_volume_.reset();

    // Re-initialize TSDF volume
    open3d::core::Device device("CPU:0");
    open3d::core::Dtype dtype = open3d::core::Dtype::Float32;

    open3d::t::geometry::TSDFVoxelGrid::Params params;
    params.voxel_size = config_.voxel_length;
    params.sdf_trunc = config_.sdf_trunc;
    params.block_resolution = 16;
    params.block_count = 10000;

    tsdf_volume_ = std::make_shared<open3d::t::geometry::TSDFVoxelGrid>(params, dtype, device);

    open3d_mesh_ = open3d::t::geometry::TriangleMesh();
    frame_count_ = 0;
    last_extract_frame_ = 0;
}

inline MeshStats DenseMeshReconstruction::GetStats() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    MeshStats result = stats_;
    result.current_voxels = tsdf_volume_->GetVoxelCount();
    return result;
}

inline void DenseMeshReconstruction::Reset() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    stats_ = MeshStats();
    frame_count_ = 0;
    last_extract_frame_ = 0;
}

inline void DenseMeshReconstruction::ResetIntegrationCounter() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    stats_.total_integrations = 0;
}

inline std::string DenseMeshReconstruction::GetTimingReport() {
#ifdef REGISTER_DENSE_TIMES
    std::shared_lock<std::shared_mutex> lock(mutex_);
    timing_stats_.CalculateStats();
    std::ostringstream ss;
    ss << "DenseMesh Timing: avg=" << timing_stats_.avg_integrate_ms
       << "ms, p99=" << timing_stats_.p99_integrate_ms
       << "ms, total=" << timing_stats_.integrate_times.size() << " frames";
    return ss.str();
#else
    return "DenseMesh Timing: (not enabled - compile with REGISTER_DENSE_TIMES)";
#endif
}

inline MeshTimingStats DenseMeshReconstruction::GetTimingStats() {
#ifdef REGISTER_DENSE_TIMES
    std::shared_lock<std::shared_mutex> lock(mutex_);
    MeshTimingStats result = timing_stats_;
    result.CalculateStats();
    return result;
#else
    return MeshTimingStats();
#endif
}

inline void DenseMeshReconstruction::OutputTimingSummary() {
#ifdef REGISTER_DENSE_TIMES
    std::cout << GetTimingReport() << std::endl;
#endif
}

inline int DenseMeshReconstruction::GetVertexCount() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (open3d_mesh_.IsEmpty()) {
        return 0;
    }
    return open3d_mesh_.GetVertexPositions().GetShape(0);
}

inline int DenseMeshReconstruction::GetTriangleCount() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (open3d_mesh_.IsEmpty()) {
        return 0;
    }
    return open3d_mesh_.GetTriangleIndices().GetShape(0);
}

inline DenseMeshReconstruction::DenseMeshReconstruction(DenseMeshReconstruction&& other) noexcept
    : tsdf_volume_(std::move(other.tsdf_volume_)),
      open3d_mesh_(std::move(other.open3d_mesh_)),
      config_(other.config_),
      stats_(other.stats_),
      frame_count_(other.frame_count_),
      last_extract_frame_(other.last_extract_frame_),
      mutex_()
{
    other.stub_frame_count_ = 0;
    other.stub_last_extract_frame_ = 0;
}

inline DenseMeshReconstruction& DenseMeshReconstruction::operator=(DenseMeshReconstruction&& other) noexcept {
    if (this != &other) {
        std::unique_lock<std::shared_mutex> lock_this(mutex_);
        std::unique_lock<std::shared_mutex> lock_other(other.mutex_);

        tsdf_volume_ = std::move(other.tsdf_volume_);
        open3d_mesh_ = std::move(other.open3d_mesh_);
        config_ = other.config_;
        stats_ = other.stats_;
        frame_count_ = other.frame_count_;
        last_extract_frame_ = other.last_extract_frame_;

        other.stub_frame_count_ = 0;
        other.stub_last_extract_frame_ = 0;
    }
    return *this;
}

#else // DENSE_MESH_ENABLED stub implementations

inline DenseMeshReconstruction::DenseMeshReconstruction(const MeshReconConfig& config)
    : stub_tsdf_volume_(nullptr),
      stub_mesh_(nullptr),
      config_(config),
      stats_(),
      stub_frame_count_(0),
      stub_last_extract_frame_(0)
{
}

inline DenseMeshReconstruction::~DenseMeshReconstruction() {
    Clear();
}

inline bool DenseMeshReconstruction::IntegrateRGBD(
    const cv::Mat&,
    const cv::Mat&,
    const Sophus::SE3f&,
    const CameraIntrinsics&)
{
    // Stub: Open3D not available, integration disabled
    return false;
}

inline bool DenseMeshReconstruction::ExtractMesh(bool) {
    // Stub: Open3D not available, no mesh extraction
    return false;
}

inline bool DenseMeshReconstruction::SaveMeshPLY(const std::string&) {
    // Stub: Open3D not available, cannot save
    return false;
}

inline bool DenseMeshReconstruction::SaveMeshOBJ(const std::string&) {
    // Stub: Open3D not available, cannot save
    return false;
}

inline void DenseMeshReconstruction::Clear() {
    stub_tsdf_volume_ = nullptr;
    stub_mesh_ = nullptr;
    stub_frame_count_ = 0;
    stub_last_extract_frame_ = 0;
}

inline MeshStats DenseMeshReconstruction::GetStats() const {
    MeshStats result = stats_;
    result.current_voxels = 0;  // No Open3D, no voxels
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
    return "DenseMesh Timing: (stub mode - no timing available)";
}

inline MeshTimingStats DenseMeshReconstruction::GetTimingStats() {
    return MeshTimingStats();
}

inline void DenseMeshReconstruction::OutputTimingSummary() {
    // No-op in stub mode
}

inline int DenseMeshReconstruction::GetVertexCount() {
    return 0;  // No mesh available in stub mode
}

inline int DenseMeshReconstruction::GetTriangleCount() {
    return 0;  // No mesh available in stub mode
}

inline DenseMeshReconstruction::DenseMeshReconstruction(DenseMeshReconstruction&& other) noexcept
    : stub_tsdf_volume_(std::move(other.stub_tsdf_volume_)),
      stub_mesh_(std::move(other.stub_mesh_)),
      config_(other.config_),
      stats_(other.stats_),
      stub_frame_count_(other.stub_frame_count_),
      stub_last_extract_frame_(other.stub_last_extract_frame_)
{
    other.stub_tsdf_volume_ = nullptr;
    other.stub_mesh_ = nullptr;
    other.stub_frame_count_ = 0;
    other.stub_last_extract_frame_ = 0;
}

inline DenseMeshReconstruction& DenseMeshReconstruction::operator=(DenseMeshReconstruction&& other) noexcept {
    if (this != &other) {
        stub_tsdf_volume_ = std::move(other.stub_tsdf_volume_);
        stub_mesh_ = std::move(other.stub_mesh_);
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

#endif // DENSE_MESH_ENABLED

}  // namespace ORB_SLAM3

#endif  // DENSE_MESH_RECONSTRUCTION_H