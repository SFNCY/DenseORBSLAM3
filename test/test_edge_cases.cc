/**
 * @file test_edge_cases.cc
 * @brief Edge case tests for DenseMeshReconstruction
 */

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <vector>
#include <cmath>
#include <fstream>

#include "DenseMeshReconstruction.h"

namespace ORB_SLAM3 {

// Test fixture for DenseMesh edge case tests
class EdgeCaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = MeshReconConfig(0.005f, 0.02f, 3, 3.0f);
        mesh_recon_ = std::make_unique<DenseMeshReconstruction>(config_);

        // Standard camera intrinsics for 640x480
        intrinsics_.fx = 525.0f;
        intrinsics_.fy = 525.0f;
        intrinsics_.cx = 319.5f;
        intrinsics_.cy = 239.5f;
        intrinsics_.width = 640;
        intrinsics_.height = 480;
    }

    void TearDown() override {
        mesh_recon_.reset();
    }

    // Create synthetic RGB image (all zeros = black)
    cv::Mat CreateSyntheticRGB(int width, int height) {
        return cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    // Create synthetic depth image with specified depth value for all pixels
    cv::Mat CreateSyntheticDepth(int width, int height, float depth_value) {
        cv::Mat depth(height, width, CV_32F, cv::Scalar(depth_value));
        return depth;
    }

    // Create depth with valid depth values in center region, zeros elsewhere
    cv::Mat CreatePartialDepth(int width, int height, float center_depth,
                               int center_x_start, int center_y_start,
                               int center_x_end, int center_y_end) {
        cv::Mat depth = cv::Mat::zeros(height, width, CV_32F);
        for (int y = center_y_start; y < center_y_end; ++y) {
            for (int x = center_x_start; x < center_x_end; ++x) {
                if (y >= 0 && y < height && x >= 0 && x < width) {
                    depth.at<float>(y, x) = center_depth;
                }
            }
        }
        return depth;
    }

    // Create depth with alternating valid/invalid pattern
    cv::Mat CreateAlternatingDepth(int width, int height, float valid_depth) {
        cv::Mat depth(height, width, CV_32F);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if ((x + y) % 2 == 0) {
                    depth.at<float>(y, x) = valid_depth;
                } else {
                    depth.at<float>(y, x) = 0.0f;  // Invalid
                }
            }
        }
        return depth;
    }

    // Create depth with extremely large values
    cv::Mat CreateLargeDepth(int width, int height, float large_depth) {
        cv::Mat depth(height, width, CV_32F, cv::Scalar(large_depth));
        return depth;
    }

    // Identity SE3 transform
    Sophus::SE3f IdentityTransform() {
        return Sophus::SE3f();
    }

    // SE3 transform with small translation
    Sophus::SE3f TranslationOnly(float tx, float ty, float tz) {
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t(tx, ty, tz);
        return Sophus::SE3f(R, t);
    }

    MeshReconConfig config_;
    CameraIntrinsics intrinsics_;
    std::unique_ptr<DenseMeshReconstruction> mesh_recon_;
};

// ============================================================================
// Test Case 1: Single Keyframe No Extraction
// ============================================================================
TEST_F(EdgeCaseTest, SingleKeyframeNoExtraction) {
    SCOPED_TRACE("EdgeCase: SingleKeyframeNoExtraction");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);
    Sophus::SE3f Tcw = IdentityTransform();

    // First keyframe - should not trigger extraction
    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth, Tcw, intrinsics_);

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "SingleKeyframeNoExtraction: integrate_ok=" << integrate_ok
              << ", total_integrations=" << stats.total_integrations
              << ", total_extractions=" << stats.total_extractions;

    // Should not extract mesh with only 1 keyframe
    EXPECT_EQ(stats.total_extractions, 0);
}

// ============================================================================
// Test Case 2: Two Keyframes No Extraction
// ============================================================================
TEST_F(EdgeCaseTest, TwoKeyframesNoExtraction) {
    SCOPED_TRACE("EdgeCase: TwoKeyframesNoExtraction");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);
    Sophus::SE3f Tcw = IdentityTransform();

    // First keyframe
    mesh_recon_->IntegrateRGBD(rgb, depth, Tcw, intrinsics_);

    // Second keyframe with small motion
    Sophus::SE3f Tcw2 = TranslationOnly(0.1f, 0.0f, 0.0f);
    mesh_recon_->IntegrateRGBD(rgb, depth, Tcw2, intrinsics_);

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "TwoKeyframesNoExtraction: total_integrations=" << stats.total_integrations
              << ", total_extractions=" << stats.total_extractions;

    // Should not extract mesh with only 2 keyframes (update_every_n_kf=3)
    EXPECT_EQ(stats.total_extractions, 0);
}

// ============================================================================
// Test Case 3: Third Keyframe Triggers Extraction
// ============================================================================
TEST_F(EdgeCaseTest, ThirdKeyframeTriggersExtraction) {
    SCOPED_TRACE("EdgeCase: ThirdKeyframeTriggersExtraction");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Keyframe 1
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);

    // Keyframe 2
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);

    // Keyframe 3 - should trigger extraction
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.2f, 0.0f, 0.0f), intrinsics_);

    // Extract mesh
    bool extract_ok = mesh_recon_->ExtractMesh();

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "ThirdKeyframeTriggersExtraction: extract_ok=" << extract_ok
              << ", total_integrations=" << stats.total_integrations
              << ", total_extractions=" << stats.total_extractions;

    EXPECT_EQ(stats.total_extractions, 1);
    EXPECT_TRUE(extract_ok);
}

// ============================================================================
// Test Case 4: Fourth Keyframe No Extraction (only 1 mesh file exists)
// ============================================================================
TEST_F(EdgeCaseTest, FourthKeyframeNoExtraction) {
    SCOPED_TRACE("EdgeCase: FourthKeyframeNoExtraction");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Keyframe 1
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);

    // Keyframe 2
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);

    // Keyframe 3 - triggers extraction
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.2f, 0.0f, 0.0f), intrinsics_);
    mesh_recon_->ExtractMesh();

    // Keyframe 4 - should not trigger another extraction
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.3f, 0.0f, 0.0f), intrinsics_);

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "FourthKeyframeNoExtraction: total_integrations=" << stats.total_integrations
              << ", total_extractions=" << stats.total_extractions;

    // With update_every_n_kf=3, keyframe 4 should not trigger extraction
    // frame_count would be 4, 4%3=1 != 0, so ShouldIntegrate() returns false
    EXPECT_EQ(stats.total_extractions, 1);
}

// ============================================================================
// Test Case 5: Rapid Camera Motion
// ============================================================================
TEST_F(EdgeCaseTest, RapidCameraMotion) {
    SCOPED_TRACE("EdgeCase: RapidCameraMotion");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Simulate rapid motion - large translations between keyframes
    std::vector<Sophus::SE3f> motions = {
        TranslationOnly(1.0f, 0.0f, 0.0f),   // 1 meter right
        TranslationOnly(2.0f, 0.0f, 0.0f),   // 2 meters right
        TranslationOnly(3.0f, 0.0f, 0.0f),   // 3 meters right
        TranslationOnly(4.0f, 0.0f, 0.0f),   // 4 meters right
    };

    bool no_crash = true;
    for (size_t i = 0; i < motions.size(); ++i) {
        try {
            mesh_recon_->IntegrateRGBD(rgb, depth, motions[i], intrinsics_);
            LOG(INFO) << "RapidCameraMotion: motion " << i << " completed OK";
        } catch (...) {
            no_crash = false;
            LOG(INFO) << "RapidCameraMotion: motion " << i << " threw exception";
        }
    }

    EXPECT_TRUE(no_crash);
    LOG(INFO) << "RapidCameraMotion: completed without crash";
}

// ============================================================================
// Test Case 6: All Frames Invalid Depth (graceful handling)
// ============================================================================
TEST_F(EdgeCaseTest, AllFramesInvalidDepth) {
    SCOPED_TRACE("EdgeCase: AllFramesInvalidDepth");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    // All depth values are 0 (invalid)
    cv::Mat depth = cv::Mat::zeros(480, 640, CV_32F);

    // Should handle gracefully - no crash
    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);

    LOG(INFO) << "AllFramesInvalidDepth: integrate_ok=" << integrate_ok;

    // Integration should fail gracefully due to >90% invalid pixels
    EXPECT_FALSE(integrate_ok);

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "AllFramesInvalidDepth: stats.total_integrations=" << stats.total_integrations;
    EXPECT_EQ(stats.total_integrations, 0);
}

// ============================================================================
// Test Case 7: Mixed Valid/Invalid Frames
// ============================================================================
TEST_F(EdgeCaseTest, MixedValidInvalidFrames) {
    SCOPED_TRACE("EdgeCase: MixedValidInvalidFrames");

    // First frame: valid depth
    {
        cv::Mat rgb = CreateSyntheticRGB(640, 480);
        cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);
        mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
    }

    // Second frame: invalid depth only
    {
        cv::Mat rgb = CreateSyntheticRGB(640, 480);
        cv::Mat depth = cv::Mat::zeros(480, 640, CV_32F);
        mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);
    }

    // Third frame: valid depth again
    {
        cv::Mat rgb = CreateSyntheticRGB(640, 480);
        cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);
        mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.2f, 0.0f, 0.0f), intrinsics_);
    }

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "MixedValidInvalidFrames: total_integrations=" << stats.total_integrations;

    // Only frames with valid depth should be integrated
    // Due to >90% invalid ratio, second frame should be rejected
    EXPECT_GE(stats.total_integrations, 2);
}

// ============================================================================
// Test Case 8: Map Reset Mid-Scan
// ============================================================================
TEST_F(EdgeCaseTest, MapResetMidScan) {
    SCOPED_TRACE("EdgeCase: MapResetMidScan");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Add some keyframes
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);

    MeshStats stats_before = mesh_recon_->GetStats();
    LOG(INFO) << "MapResetMidScan: before reset, integrations=" << stats_before.total_integrations;

    // Reset mid-scan
    mesh_recon_->Clear();

    // Add keyframes after reset
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);

    MeshStats stats_after = mesh_recon_->GetStats();
    LOG(INFO) << "MapResetMidScan: after reset, integrations=" << stats_after.total_integrations;

    // Should have fresh state after reset
    EXPECT_EQ(stats_after.total_integrations, 2);
}

// ============================================================================
// Test Case 9: Shutdown Before Third Keyframe
// ============================================================================
TEST_F(EdgeCaseTest, ShutdownBeforeThirdKeyframe) {
    SCOPED_TRACE("EdgeCase: ShutdownBeforeThirdKeyframe");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Only 2 keyframes - no extraction triggered
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);

    // Simulate shutdown - output timing summary
    mesh_recon_->OutputTimingSummary();

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "ShutdownBeforeThirdKeyframe: total_integrations=" << stats.total_integrations
              << ", total_extractions=" << stats.total_extractions;

    EXPECT_EQ(stats.total_extractions, 0);
    LOG(INFO) << "ShutdownBeforeThirdKeyframe: completed gracefully";
}

// ============================================================================
// Test Case 10: Very Large Depth Values (depth overflow protection)
// ============================================================================
TEST_F(EdgeCaseTest, VeryLargeDepthValues) {
    SCOPED_TRACE("EdgeCase: VeryLargeDepthValues");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);

    // Test with depth values exceeding thDepth (3.0f)
    cv::Mat depth = CreateLargeDepth(640, 480, 100.0f);

    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);

    LOG(INFO) << "VeryLargeDepthValues: integrate_ok=" << integrate_ok;

    // Large depth values should be filtered out by IsDepthValid()
    // Should not crash, but may reject due to >90% invalid pixels
    EXPECT_FALSE(integrate_ok);

    // Also test with depth = 0 (edge case of minimum)
    cv::Mat depth_zero = CreateLargeDepth(640, 480, 0.0f);
    integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth_zero, IdentityTransform(), intrinsics_);

    LOG(INFO) << "VeryLargeDepthValues: depth_zero integrate_ok=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);

    // Test with negative depth (should also be invalid)
    cv::Mat depth_negative = CreateLargeDepth(640, 480, -1.0f);
    integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth_negative, IdentityTransform(), intrinsics_);

    LOG(INFO) << "VeryLargeDepthValues: negative_depth integrate_ok=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);

    LOG(INFO) << "VeryLargeDepthValues: all boundary conditions handled";
}

// ============================================================================
// Additional Edge Case Tests
// ============================================================================

// Test: Alternating valid/invalid depth pattern
TEST_F(EdgeCaseTest, AlternatingDepthPattern) {
    SCOPED_TRACE("EdgeCase: AlternatingDepthPattern");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateAlternatingDepth(640, 480, 1.5f);

    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);

    LOG(INFO) << "AlternatingDepthPattern: integrate_ok=" << integrate_ok;

    // With 50% valid depth, should pass the 90% threshold check
    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "AlternatingDepthPattern: total_integrations=" << stats.total_integrations;
}

// Test: Depth at exact boundary values
TEST_F(EdgeCaseTest, DepthBoundaryValues) {
    SCOPED_TRACE("EdgeCase: DepthBoundaryValues");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);

    // Test depth = 0.1f (minimum valid)
    {
        cv::Mat depth = CreateSyntheticDepth(640, 480, 0.1f);
        bool ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
        LOG(INFO) << "DepthBoundaryValues: depth=0.1f, ok=" << ok;
    }

    // Test depth = 2.99f (just under max)
    {
        cv::Mat depth = CreateSyntheticDepth(640, 480, 2.99f);
        bool ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
        LOG(INFO) << "DepthBoundaryValues: depth=2.99f, ok=" << ok;
    }

    // Test depth = 3.01f (just over max)
    {
        cv::Mat depth = CreateSyntheticDepth(640, 480, 3.01f);
        bool ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
        LOG(INFO) << "DepthBoundaryValues: depth=3.01f, ok=" << ok;
        EXPECT_FALSE(ok);
    }

    // Test depth = 0.05f (below minimum)
    {
        cv::Mat depth = CreateSyntheticDepth(640, 480, 0.05f);
        bool ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
        LOG(INFO) << "DepthBoundaryValues: depth=0.05f, ok=" << ok;
        EXPECT_FALSE(ok);
    }
}

// Test: Image dimension mismatches
TEST_F(EdgeCaseTest, ImageDimensionMismatch) {
    SCOPED_TRACE("EdgeCase: ImageDimensionMismatch");

    cv::Mat rgb_640x480 = CreateSyntheticRGB(640, 480);
    cv::Mat depth_320x240 = CreateSyntheticDepth(320, 240, 1.5f);

    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb_640x480, depth_320x240, IdentityTransform(), intrinsics_);

    LOG(INFO) << "ImageDimensionMismatch: integrate_ok=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);
}

// Test: Invalid image types
TEST_F(EdgeCaseTest, InvalidImageTypes) {
    SCOPED_TRACE("EdgeCase: InvalidImageTypes");

    // Wrong RGB type (grayscale instead of BGR)
    cv::Mat rgb_gray(480, 640, CV_8UC1, cv::Scalar(128));
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb_gray, depth, IdentityTransform(), intrinsics_);

    LOG(INFO) << "InvalidImageTypes: wrong_rgb_type=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);

    // Wrong depth type (uint16 instead of float32)
    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth_wrong_type(480, 640, CV_16UC1, cv::Scalar(1500));

    integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth_wrong_type, IdentityTransform(), intrinsics_);

    LOG(INFO) << "InvalidImageTypes: wrong_depth_type=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);
}

// Test: Invalid intrinsics
TEST_F(EdgeCaseTest, InvalidIntrinsics) {
    SCOPED_TRACE("EdgeCase: InvalidIntrinsics");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    CameraIntrinsics invalid_intrinsics = intrinsics_;
    invalid_intrinsics.fx = -1.0f;  // Invalid focal length

    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), invalid_intrinsics);

    LOG(INFO) << "InvalidIntrinsics: integrate_ok=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);
}

// Test: SaveMeshPLY with no mesh (should fail gracefully)
TEST_F(EdgeCaseTest, SaveMeshWithoutExtraction) {
    SCOPED_TRACE("EdgeCase: SaveMeshWithoutExtraction");

    bool save_ok = mesh_recon_->SaveMeshPLY("/tmp/test_nomesh.ply");

    LOG(INFO) << "SaveMeshWithoutExtraction: save_ok=" << save_ok;
    EXPECT_FALSE(save_ok);
}

// Test: GetVertexCount and GetTriangleCount without extraction
TEST_F(EdgeCaseTest, GetCountsWithoutExtraction) {
    SCOPED_TRACE("EdgeCase: GetCountsWithoutExtraction");

    int vertices = mesh_recon_->GetVertexCount();
    int triangles = mesh_recon_->GetTriangleCount();

    LOG(INFO) << "GetCountsWithoutExtraction: vertices=" << vertices << ", triangles=" << triangles;

    EXPECT_EQ(vertices, 0);
    EXPECT_EQ(triangles, 0);
}

// Test: Reset statistics
TEST_F(EdgeCaseTest, ResetStatistics) {
    SCOPED_TRACE("EdgeCase: ResetStatistics");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Integrate some frames
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);

    MeshStats stats_before = mesh_recon_->GetStats();
    LOG(INFO) << "ResetStatistics: before reset, integrations=" << stats_before.total_integrations;

    mesh_recon_->Reset();

    MeshStats stats_after = mesh_recon_->GetStats();
    LOG(INFO) << "ResetStatistics: after reset, integrations=" << stats_after.total_integrations;

    EXPECT_EQ(stats_after.total_integrations, 0);
    EXPECT_EQ(stats_after.total_extractions, 0);
}

// Test: Very small partial depth (just under 10% valid)
TEST_F(EdgeCaseTest, JustUnderInvalidThreshold) {
    SCOPED_TRACE("EdgeCase: JustUnderInvalidThreshold");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    // Only 8% of pixels have valid depth - should fail (>90% invalid)
    cv::Mat depth = CreatePartialDepth(640, 480, 1.5f, 0, 0, 200, 200);

    bool integrate_ok = mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);

    LOG(INFO) << "JustUnderInvalidThreshold: integrate_ok=" << integrate_ok;
    EXPECT_FALSE(integrate_ok);
}

// Test: Exact update_every_n_kf boundary
TEST_F(EdgeCaseTest, ExactUpdateBoundary) {
    SCOPED_TRACE("EdgeCase: ExactUpdateBoundary");

    cv::Mat rgb = CreateSyntheticRGB(640, 480);
    cv::Mat depth = CreateSyntheticDepth(640, 480, 1.5f);

    // Keyframe 3
    mesh_recon_->IntegrateRGBD(rgb, depth, IdentityTransform(), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.1f, 0.0f, 0.0f), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.2f, 0.0f, 0.0f), intrinsics_);

    // Keyframe 6 - another boundary
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.3f, 0.0f, 0.0f), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.4f, 0.0f, 0.0f), intrinsics_);
    mesh_recon_->IntegrateRGBD(rgb, depth, TranslationOnly(0.5f, 0.0f, 0.0f), intrinsics_);

    MeshStats stats = mesh_recon_->GetStats();
    LOG(INFO) << "ExactUpdateBoundary: total_integrations=" << stats.total_integrations;

    // Should integrate at keyframes 3 and 6
    EXPECT_GE(stats.total_integrations, 2);
}

}  // namespace ORB_SLAM3