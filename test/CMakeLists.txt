# Test framework using Google Test
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
if(WIN32)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()

FetchContent_MakeAvailable(googletest)

enable_testing()

# Main entry point for all tests
add_executable(test_example test_main.cc test_example.cc)
target_link_libraries(test_example
    ${PROJECT_NAME}
    GTest::gtest
)

# Edge case tests for DenseMeshReconstruction
add_executable(test_edge_cases test_main.cc test_edge_cases.cc)
target_link_libraries(test_edge_cases
    ${PROJECT_NAME}
    GTest::gtest
)

# DenseMeshReconstruction tests
add_executable(test_dense_mesh_reconstruction test_main.cc test_dense_mesh_reconstruction.cc)
target_link_libraries(test_dense_mesh_reconstruction
    ${PROJECT_NAME}
    GTest::gtest
)

# Regression tests for Dense Mesh feature
add_executable(test_regression test_main.cc test_regression.cc)
target_link_libraries(test_regression
    ${PROJECT_NAME}
    GTest::gtest
)

# Integration tests for DenseMesh end-to-end workflow
add_executable(test_integration_dense_mesh test_main.cc test_integration_dense_mesh.cc)
target_link_libraries(test_integration_dense_mesh
    ${PROJECT_NAME}
    GTest::gtest
)

include(GoogleTest)
gtest_discover_tests(test_example)
gtest_discover_tests(test_edge_cases)
gtest_discover_tests(test_dense_mesh_reconstruction)
gtest_discover_tests(test_regression)
gtest_discover_tests(test_integration_dense_mesh)