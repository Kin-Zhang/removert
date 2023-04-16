
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */

#pragma once

#include <iostream>
#include <pcl/point_types.h>
#include <string>

#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

// CHANGE Point Type Here!!! If you want to use XYZI, change to pcl::PointXYZI
// typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI PointType;
// typedef pcl::PointXYZRGB PointType;

namespace common {
struct Config {
    /**< Parameters of MapUpdater*/
    std::vector<double> remove_resolution_list_ = {2.5, 2.0, 1.5};
    std::pair<float, float> kFOV = {60.0, 60.0};
    bool verbose_ = false; // print out logs
    int kNumOmpCores = 8;

    std::string mode = "naive";
    bool replace_intensity = false;

    // NOT recommend to use for under 5 million points map input (becausing
    // not-using is just faster)
    const bool kUseSubsetMapCloud = false;
    const float kBallSize = 80.0; // meter

    float downsample_voxel_size_ = 0.05;
};

} // namespace common