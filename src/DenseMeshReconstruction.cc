/**
 * @file DenseMeshReconstruction.cc
 * @brief Dense mesh reconstruction using Open3D VoxelBlockGrid TSDF fusion
 *        (Open3D 0.19.0 API). All Open3D-dependent code is isolated here
 *        to avoid GL/glew.h conflicts with Pangolin.
 */
#ifdef DENSE_MESH_ENABLED

#include "DenseMeshReconstruction.h"

#include <open3d/Open3D.h>

#include <opencv2/imgproc.hpp>
#include <iostream>

namespace ORB_SLAM3 {

// ============================================================================
// PIMPL implementation struct - hides all Open3D types from the header
// ============================================================================
struct DenseMeshReconstruction::Impl {
    // TSDF volume (replaces old TSDFVoxelGrid with VoxelBlockGrid)
    std::shared_ptr<open3d::t::geometry::VoxelBlockGrid> tsdf_volume_;

    // Extracted mesh
    open3d::t::geometry::TriangleMesh open3d_mesh_;

    // Configuration
    MeshReconConfig config_;

    // Statistics
    MeshStats stats_;

#ifdef REGISTER_DENSE_TIMES
    MeshTimingStats timing_stats_;
#endif

    // Frame counting for integration throttling
    int frame_count_ = 0;
    int last_extract_frame_ = 0;

    // Thread safety
    mutable std::shared_mutex mutex_;

    // Extracted mesh data (Eigen format for external access)
    std::vector<Eigen::Vector3d> vertices_;
    std::vector<Eigen::Vector3i> triangles_;
    std::vector<Eigen::Vector3d> vertex_colors_;

    bool volume_initialized_ = false;

    void InitVolume() {
        if (volume_initialized_) return;

        // Open3D 0.19 VoxelBlockGrid constructor:
        //   attr_names = {"tsdf", "weight", "color"}
        //   attr_dtypes = {Float32, UInt16, UInt16}
        //   attr_channels = {{1}, {1}, {3}}
        //   voxel_size, block_resolution, block_count, device, backend
        tsdf_volume_ = std::make_shared<open3d::t::geometry::VoxelBlockGrid>(
            std::vector<std::string>{"tsdf", "weight", "color"},
            std::vector<open3d::core::Dtype>{
                open3d::core::Float32,
                open3d::core::UInt16,
                open3d::core::UInt16},
            std::vector<open3d::core::SizeVector>{{1}, {1}, {3}},
            config_.voxel_length,
            16,     // block_resolution
            10000,  // block_count
            open3d::core::Device("CPU:0"),
            open3d::core::HashBackendType::Default);

        volume_initialized_ = true;
    }

    bool ShouldIntegrate() const {
        return (frame_count_ % config_.update_every_n_kf) == 0;
    }

    bool IsDepthValid(float z_mm) const {
        return z_mm > 100.0f && z_mm < config_.thDepth * 1000.0f;
    }

    /// Compute trunc_voxel_multiplier from sdf_trunc and voxel_length.
    float GetTruncVoxelMultiplier() const {
        if (config_.voxel_length <= 0.0f) return 8.0f;
        return config_.sdf_trunc / config_.voxel_length;
    }

    bool IntegrateFrame(const cv::Mat& rgb,
                        const cv::Mat& depth_filtered,
                        const Sophus::SE3f& Tcw,
                        const CameraIntrinsics& intrinsics) {
        open3d::core::Device device("CPU:0");

        // RealSense D435i outputs RGB8 format directly, no conversion needed
        // Open3D SLAM mode expects (uint16 depth in mm, uint8 color)
        open3d::core::Tensor rgb_tensor(
            rgb.ptr<uint8_t>(),
            {static_cast<int64_t>(rgb.rows),
             static_cast<int64_t>(rgb.cols), 3},
            open3d::core::Dtype::UInt8, device);

        open3d::core::Tensor depth_tensor(
            depth_filtered.ptr<uint16_t>(),
            {static_cast<int64_t>(depth_filtered.rows),
             static_cast<int64_t>(depth_filtered.cols)},
            open3d::core::Dtype::UInt16, device);

        open3d::t::geometry::Image o3d_rgb(rgb_tensor);
        open3d::t::geometry::Image o3d_depth(depth_tensor);

        Eigen::Matrix3d K;
        K << intrinsics.fx, 0.0, intrinsics.cx,
             0.0, intrinsics.fy, intrinsics.cy,
             0.0, 0.0, 1.0;
        open3d::core::Tensor intrinsic_tensor =
            open3d::core::eigen_converter::EigenMatrixToTensor(K);

        // ORB-SLAM3 convention: Tcw = world→camera (camera pose)
        // Open3D expects extrinsic = world→camera
        Eigen::Matrix4d extrinsic_mat = Tcw.matrix().cast<double>();
        open3d::core::Tensor extrinsic_tensor =
            open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_mat);

        // depth in mm → depth_scale = 1000.0f (default)
        float depth_max = config_.thDepth;
        float trunc_mult = GetTruncVoxelMultiplier();

        auto frustum_block_coords = tsdf_volume_->GetUniqueBlockCoordinates(
            o3d_depth, intrinsic_tensor, extrinsic_tensor,
            1000.0f, depth_max, trunc_mult);

        tsdf_volume_->Integrate(frustum_block_coords,
                                o3d_depth, o3d_rgb,
                                intrinsic_tensor, extrinsic_tensor,
                                1000.0f, depth_max, trunc_mult);

        stats_.total_integrations++;
        frame_count_++;

        return true;
    }
};

// ============================================================================
// Constructor / Destructor
// ============================================================================
DenseMeshReconstruction::DenseMeshReconstruction(const MeshReconConfig& config)
    : impl_(std::make_unique<Impl>())
{
    impl_->config_ = config;
}

DenseMeshReconstruction::~DenseMeshReconstruction() = default;

// ============================================================================
// Move operations
// ============================================================================
DenseMeshReconstruction::DenseMeshReconstruction(DenseMeshReconstruction&& other) noexcept
    : impl_(std::move(other.impl_))
{}

DenseMeshReconstruction& DenseMeshReconstruction::operator=(
    DenseMeshReconstruction&& other) noexcept
{
    if (this != &other) {
        impl_ = std::move(other.impl_);
    }
    return *this;
}

// ============================================================================
// IntegrateRGBD
// ============================================================================
bool DenseMeshReconstruction::IntegrateRGBD(
    const cv::Mat& rgb,
    const cv::Mat& depth,
    const Sophus::SE3f& Tcw,
    const CameraIntrinsics& intrinsics)
{
    if (rgb.type() != CV_8UC3) return false;
    if (depth.type() != CV_16U) return false;
    if (rgb.rows != depth.rows || rgb.cols != depth.cols) return false;
    if (intrinsics.fx <= 0 || intrinsics.fy <= 0) return false;
    if (rgb.rows != intrinsics.height || rgb.cols != intrinsics.width) return false;

    cv::Mat depth_filtered = depth.clone();
    int total_pixels = depth.rows * depth.cols;
    int invalid_pixels = 0;
    for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
            uint16_t z_mm = depth.at<uint16_t>(v, u);
            if (!impl_->IsDepthValid(static_cast<float>(z_mm))) {
                depth_filtered.at<uint16_t>(v, u) = 0;
                invalid_pixels++;
            }
        }
    }
    float invalid_ratio = static_cast<float>(invalid_pixels) /
                          static_cast<float>(total_pixels);
    if (invalid_ratio > 0.90f) return false;

    std::unique_lock<std::shared_mutex> lock(impl_->mutex_);
    impl_->InitVolume();

#ifdef REGISTER_DENSE_TIMES
    auto integrate_start = std::chrono::steady_clock::now();
#endif

    bool result = impl_->IntegrateFrame(rgb, depth_filtered, Tcw, intrinsics);

#ifdef REGISTER_DENSE_TIMES
    auto integrate_end = std::chrono::steady_clock::now();
    auto integrate_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        integrate_end - integrate_start);
    double integrate_ms = static_cast<double>(integrate_duration.count());

    impl_->timing_stats_.last_integrate_ms = integrate_ms;
    impl_->timing_stats_.integrate_times.push_back(integrate_ms);
    impl_->timing_stats_.CalculateStats();

    if (integrate_ms > 100.0) {
        std::cout << "DenseMesh WARNING: IntegrateRGBD took " << integrate_ms
                  << "ms (>100ms threshold)" << std::endl;
    }
#endif

    return result;
}

// ============================================================================
// ExtractMesh
// ============================================================================
bool DenseMeshReconstruction::ExtractMesh(bool incremental_only) {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex_);

    if (!impl_->volume_initialized_) return false;
    if (impl_->tsdf_volume_->GetHashMap().Size() == 0) return false;

    auto start_time = std::chrono::steady_clock::now();
    size_t num_blocks = impl_->tsdf_volume_->GetHashMap().Size();
    std::cout << "[DenseMesh] Running MarchingCubes on " << num_blocks
              << " blocks..." << std::endl;

    impl_->open3d_mesh_ = impl_->tsdf_volume_->ExtractTriangleMesh();

    if (impl_->open3d_mesh_.IsEmpty()) {
        std::cout << "[DenseMesh] MarchingCubes produced empty mesh" << std::endl;
        return false;
    }

    auto& verts = impl_->open3d_mesh_.GetVertexPositions();
    auto& tris  = impl_->open3d_mesh_.GetTriangleIndices();
    auto& cols  = impl_->open3d_mesh_.GetVertexColors();

    int nv = static_cast<int>(verts.GetShape(0));
    int nt = static_cast<int>(tris.GetShape(0));

    impl_->vertices_.resize(nv);
    impl_->triangles_.resize(nt);
    impl_->vertex_colors_.resize(nv);

    float* vd = verts.GetDataPtr<float>();
    for (int i = 0; i < nv; ++i)
        impl_->vertices_[i] = Eigen::Vector3d(vd[i*3], vd[i*3+1], vd[i*3+2]);

    int* td = tris.GetDataPtr<int>();
    for (int i = 0; i < nt; ++i)
        impl_->triangles_[i] = Eigen::Vector3i(td[i*3], td[i*3+1], td[i*3+2]);

    if (cols.GetShape(0) > 0) {
        float* cd = cols.GetDataPtr<float>();
        for (int i = 0; i < nv; ++i)
            impl_->vertex_colors_[i] = Eigen::Vector3d(cd[i*3], cd[i*3+1], cd[i*3+2]);
    }

    ApplyBilateralFilter(impl_->config_.bilateral_spatial_sigma,
                         impl_->config_.bilateral_color_sigma);

    auto end_time = std::chrono::steady_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();
    std::cout << "[DenseMesh] Mesh extracted: " << nv << " verts, "
              << nt << " tris in " << ms << " ms" << std::endl;

#ifdef REGISTER_DENSE_TIMES
    impl_->timing_stats_.last_extract_ms = ms;
    impl_->timing_stats_.extract_times.push_back(ms);
#endif
    impl_->stats_.last_extract_time_ms = ms;
    impl_->stats_.current_voxels = static_cast<int>(num_blocks);
    impl_->stats_.total_extractions++;

    return true;
}

// ============================================================================
// SaveMeshPLY
// ============================================================================
bool DenseMeshReconstruction::SaveMeshPLY(const std::string& filename) {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);

    if (impl_->open3d_mesh_.IsEmpty()) return false;

    // Check file writable
    std::ofstream test_file(filename);
    if (!test_file.is_open()) return false;
    test_file.close();

    // Open3D 0.19: ToLegacy() instead of ToLegacyTriangleMesh()
    auto legacy_mesh = impl_->open3d_mesh_.ToLegacy();
    return open3d::io::WriteTriangleMesh(filename, legacy_mesh);
}

// ============================================================================
// SaveMeshOBJ
// ============================================================================
bool DenseMeshReconstruction::SaveMeshOBJ(const std::string& filename) {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);

    if (impl_->open3d_mesh_.IsEmpty()) return false;

    // Check file writable
    std::ofstream test_file(filename);
    if (!test_file.is_open()) return false;
    test_file.close();

    auto legacy_mesh = impl_->open3d_mesh_.ToLegacy();
    return open3d::io::WriteTriangleMesh(filename, legacy_mesh);
}

// ============================================================================
// Clear
// ============================================================================
void DenseMeshReconstruction::Clear() {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex_);

    // Destroy and recreate the TSDF volume
    impl_->tsdf_volume_.reset();
    impl_->volume_initialized_ = false;

    // Re-initialize
    impl_->InitVolume();

    impl_->open3d_mesh_ = open3d::t::geometry::TriangleMesh();
    impl_->frame_count_ = 0;
    impl_->last_extract_frame_ = 0;
}

// ============================================================================
// Statistics accessors
// ============================================================================
MeshStats DenseMeshReconstruction::GetStats() const {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);
    MeshStats result = impl_->stats_;
    if (impl_->volume_initialized_) {
        result.current_voxels = static_cast<int>(
            impl_->tsdf_volume_->GetHashMap().Size());
    }
    return result;
}

void DenseMeshReconstruction::Reset() {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex_);
    impl_->stats_ = MeshStats();
    impl_->frame_count_ = 0;
    impl_->last_extract_frame_ = 0;
}

void DenseMeshReconstruction::ResetIntegrationCounter() {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex_);
    impl_->stats_.total_integrations = 0;
}

// ============================================================================
// Timing
// ============================================================================
std::string DenseMeshReconstruction::GetTimingReport() {
#ifdef REGISTER_DENSE_TIMES
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);
    impl_->timing_stats_.CalculateStats();
    std::ostringstream ss;
    ss << "DenseMesh Timing: avg=" << impl_->timing_stats_.avg_integrate_ms
       << "ms, p99=" << impl_->timing_stats_.p99_integrate_ms
       << "ms, total=" << impl_->timing_stats_.integrate_times.size() << " frames";
    return ss.str();
#else
    return "DenseMesh Timing: (not enabled - compile with REGISTER_DENSE_TIMES)";
#endif
}

MeshTimingStats DenseMeshReconstruction::GetTimingStats() {
#ifdef REGISTER_DENSE_TIMES
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);
    MeshTimingStats result = impl_->timing_stats_;
    result.CalculateStats();
    return result;
#else
    return MeshTimingStats();
#endif
}

void DenseMeshReconstruction::OutputTimingSummary() {
#ifdef REGISTER_DENSE_TIMES
    std::cout << GetTimingReport() << std::endl;
#endif
}

// ============================================================================
// Vertex/Triangle count
// ============================================================================
int DenseMeshReconstruction::GetVertexCount() {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);
    if (impl_->open3d_mesh_.IsEmpty()) return 0;
    return static_cast<int>(impl_->open3d_mesh_.GetVertexPositions().GetShape(0));
}

int DenseMeshReconstruction::GetTriangleCount() {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);
    if (impl_->open3d_mesh_.IsEmpty()) return 0;
    return static_cast<int>(impl_->open3d_mesh_.GetTriangleIndices().GetShape(0));
}

void DenseMeshReconstruction::GetMeshData(std::vector<Eigen::Vector3d>& vertices,
                                           std::vector<Eigen::Vector3i>& triangles,
                                           std::vector<Eigen::Vector3d>& colors) const {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex_);
    vertices = impl_->vertices_;
    triangles = impl_->triangles_;
    colors = impl_->vertex_colors_;
}

void DenseMeshReconstruction::ApplyBilateralFilter(float spatial_sigma, float color_sigma) {
    int n = static_cast<int>(impl_->vertices_.size());
    if (n == 0 || impl_->vertex_colors_.empty()) return;

    open3d::geometry::PointCloud pcd;
    pcd.points_.resize(n);
    for (int i = 0; i < n; ++i)
        pcd.points_[i] = impl_->vertices_[i];

    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(pcd);

    std::vector<Eigen::Vector3d> filtered(n);

    double spatial_var = 2.0 * spatial_sigma * spatial_sigma;
    double color_var = 2.0 * color_sigma * color_sigma;
    double search_radius = 3.0 * spatial_sigma;

    for (int i = 0; i < n; ++i) {
        std::vector<int> indices;
        std::vector<double> dists;
        kdtree.SearchRadius(pcd.points_[i], search_radius, indices, dists);

        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        double weight_sum = 0.0;
        Eigen::Vector3d ci = impl_->vertex_colors_[i];

        for (size_t j = 0; j < indices.size(); ++j) {
            double sw = std::exp(-dists[j] / spatial_var);
            Eigen::Vector3d diff = impl_->vertex_colors_[indices[j]] - ci;
            double cw = std::exp(-diff.squaredNorm() / color_var);
            double w = sw * cw;
            sum += w * impl_->vertex_colors_[indices[j]];
            weight_sum += w;
        }

        if (weight_sum > 0.0)
            filtered[i] = sum / weight_sum;
        else
            filtered[i] = ci;
    }

    impl_->vertex_colors_ = std::move(filtered);

    std::cout << "[DenseMesh] Bilateral filter: spatial=" << spatial_sigma
              << "m, color=" << color_sigma << ", " << n << " vertices" << std::endl;
}

}  // namespace ORB_SLAM3

#endif  // DENSE_MESH_ENABLED
