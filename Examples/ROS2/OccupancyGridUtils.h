#ifndef OCCUPANCY_GRID_UTILS_H
#define OCCUPANCY_GRID_UTILS_H

#include <vector>
#include <fstream>
#include <cmath>
#include <string>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>

struct GridPoint3D {
    float x, y, z;
    GridPoint3D() : x(0), y(0), z(0) {}
    GridPoint3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

struct OccupancyCell {
    int8_t value;
};

struct OccupancyGrid {
    std::vector<OccupancyCell> cells;
    unsigned int width;
    unsigned int height;
    double resolution;
    double originX;
    double originY;
    std::string frame_id;
};

/**
 * @brief Projects a 3D point cloud onto a 2D occupancy grid (ROS XY plane).
 *
 * Input point cloud must be in ROS coordinate frame:
 *   - X = forward (direction of travel)
 *   - Y = left
 *   - Z = up (gravity axis)
 *
 * The function projects the XY plane (ground plane) to a 2D occupancy grid.
 * The minHeight/maxHeight parameters filter points along the Z (up) axis,
 * keeping only points whose Z coordinate is within [minHeight, maxHeight].
 * This is useful for filtering out ground points or obstacles at specific heights.
 *
 * @param points        Vector of 3D points in ROS coordinate frame
 * @param resolution    Grid cell size in world units (default: 0.05m)
 * @param minHeight     Minimum Z height to consider (default: -infinity)
 * @param maxHeight     Maximum Z height to consider (default: +infinity)
 * @param occupiedThreshold Minimum point count to mark cell as occupied (default: 1)
 * @return OccupancyGrid 2D grid projected from XY plane
 */
inline OccupancyGrid ProjectPointCloudToGrid(
    const std::vector<GridPoint3D>& points,
    double resolution = 0.05,
    float minHeight = -std::numeric_limits<float>::infinity(),
    float maxHeight = std::numeric_limits<float>::infinity(),
    int occupiedThreshold = 1)
{
    OccupancyGrid grid;
    grid.resolution = resolution;
    grid.frame_id = "map";

    if (points.empty()) {
        grid.width = 0;
        grid.height = 0;
        return grid;
    }

    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minZWorld = std::numeric_limits<float>::max();
    float maxZWorld = std::numeric_limits<float>::lowest();

    for (const auto& pt : points) {
        if (pt.y < minHeight || pt.y > maxHeight)
            continue;
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minZWorld = std::min(minZWorld, pt.z);
        maxZWorld = std::max(maxZWorld, pt.z);
    }

    if (minX == std::numeric_limits<float>::max()) {
        grid.width = 0;
        grid.height = 0;
        return grid;
    }

    const double epsilon = 1e-6;
    minX -= epsilon;
    maxX += epsilon;
    minZWorld -= epsilon;
    maxZWorld += epsilon;

    grid.width = static_cast<unsigned int>(std::ceil((maxX - minX) / resolution));
    grid.height = static_cast<unsigned int>(std::ceil((maxZWorld - minZWorld) / resolution));
    // Origin maps ROS frame to grid coordinates:
    // grid.originX = ROS X (forward direction) -> maps to grid column index (ix)
    // grid.originY = ROS Z (up) -> but semantically stores ROS Y (left) for grid row index (iz)
    // This is because we project the ROS XY ground plane onto the grid, using Z as height filter.
    grid.originX = minX;
    grid.originY = minZWorld;

    if (grid.width == 0 || grid.height == 0) {
        grid.width = 0;
        grid.height = 0;
        grid.cells.clear();
        return grid;
    }

    grid.cells.resize(grid.width * grid.height, OccupancyCell{-1});
    std::vector<int> pointCounts(grid.width * grid.height, 0);

    for (const auto& pt : points) {
        if (pt.y < minHeight || pt.y > maxHeight)
            continue;

        int ix = static_cast<int>(std::floor((pt.x - minX) / resolution));
        int iz = static_cast<int>(std::floor((pt.z - minZWorld) / resolution));

        ix = std::max(0, std::min(ix, static_cast<int>(grid.width) - 1));
        iz = std::max(0, std::min(iz, static_cast<int>(grid.height) - 1));

        int idx = iz * static_cast<int>(grid.width) + ix;
        pointCounts[idx]++;
    }

    for (size_t i = 0; i < grid.cells.size(); ++i) {
        grid.cells[i].value = (pointCounts[i] >= occupiedThreshold) ? 100 : 0;
    }

    return grid;
}

inline bool WriteOccupancyGridPGM(const OccupancyGrid& grid, const std::string& filename) {
    if (grid.width == 0 || grid.height == 0) {
        return false;
    }

    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "P5\n";
    ofs << "# Created by ORB-SLAM3 OccupancyGridUtils\n";
    ofs << grid.width << " " << grid.height << "\n";
    ofs << "255\n";

    for (unsigned int iz = 0; iz < grid.height; ++iz) {
        for (unsigned int ix = 0; ix < grid.width; ++ix) {
            size_t idx = iz * grid.width + ix;
            int8_t cellValue = grid.cells[idx].value;

            uint8_t pixel;
            if (cellValue == 100) {
                pixel = 0;
            } else if (cellValue == 0) {
                pixel = 255;
            } else {
                pixel = 205;
            }
            ofs.write(reinterpret_cast<char*>(&pixel), 1);
        }
    }

    ofs.close();
    return true;
}

inline bool WriteOccupancyGridYAML(const OccupancyGrid& grid,
                                   const std::string& imageFilename,
                                   const std::string& yamlFilename) {
    if (grid.width == 0 || grid.height == 0) {
        return false;
    }

    std::ofstream ofs(yamlFilename);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "image: " << imageFilename << "\n";
    ofs << "resolution: " << std::fixed << std::setprecision(6) << grid.resolution << "\n";
    ofs << "origin: [" << grid.originX << ", " << grid.originY << ", 0.0]\n";
    ofs << "occupied_thresh: 0.65\n";
    ofs << "free_thresh: 0.196\n";
    ofs << "negate: 0\n";
    ofs << "mode: trinary\n";

    ofs.close();
    return true;
}

#endif
