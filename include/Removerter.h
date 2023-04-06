/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-06 17:34
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#pragma once

// function lib
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include "timing.hpp"
#include "utils.h"

namespace removert {
class MapUpdater {
 public:
  MapUpdater(const std::string &config_file_path);
  virtual ~MapUpdater() = default;

  void setRawMap(const pcl::PointCloud<PointT>::Ptr &rawmap);
  void run(const pcl::PointCloud<PointT>::Ptr &single_pc);
  void saveMap(const std::string &map_path);
  void setConfig();
  ufo::Timing timing;

 private:
  common::Config cfg_;
  YAML::Node yconfig;
};
}  // namespace removert