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
#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

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

    void setRawMap(const pcl::PointCloud<PointType>::Ptr &rawmap);
    void run(const pcl::PointCloud<PointType>::Ptr &single_pc, float _rm_res);
    void saveMap(const std::string &map_path);
    void setConfig();
    void parseDynamicIdx2PointCloud();

    common::Config cfg_;
    ufo::Timing timing;

  private:
    YAML::Node yconfig;
    void removeOnce(float _rm_res);
    float curr_res_alpha_ = 0.0;
    float x_curr, y_curr, z_curr;
    std::vector<int> dynamic_point_indexes;

    cv::Mat scan2RangeImg(const pcl::PointCloud<PointType>::Ptr &_scan,
                          const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                          const std::pair<int, int> _rimg_size);
    void GetPointcloudUsingPtIdx(std::vector<int> &_point_indexes, pcl::PointCloud<PointType>::Ptr &_pointcloud,
                                 pcl::PointCloud<PointType>::Ptr &_pointcloud_out, bool _is_negative);
    std::vector<int> calcDescrepancyAndParseDynamicPointIdx(const cv::Mat &_scan_rimg, const cv::Mat &_diff_rimg,
                                                            const cv::Mat &_map_rimg_ptidx);

    pcl::PointCloud<PointType>::Ptr map_arranged_; // raw map
    pcl::PointCloud<PointType>::Ptr map_cleaned_;  // cleaned map
    pcl::PointCloud<PointType>::Ptr map_dynamic_;  // dynamic map

    const float kFlagNoPOINT = 10000.0;       // no point constant, 10000 has no meaning, but must be
                                              // larger than the maximum scan range (e.g., 200 meters)
    const float kValidDiffUpperBound = 200.0; // must smaller than kFlagNoPOINT

    std::pair<cv::Mat, cv::Mat>
    map2RangeImg(const pcl::PointCloud<PointType>::Ptr &_scan,
                 const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                 const std::pair<int, int> _rimg_size);
};

// utility functions
std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio) {
    // default is 1 deg x 1 deg
    float alpha_vfov = _resize_ratio;
    float alpha_hfov = _resize_ratio;

    float V_FOV = _fov.first;
    float H_FOV = _fov.second;

    int NUM_RANGE_IMG_ROW = std::round(V_FOV * alpha_vfov);
    int NUM_RANGE_IMG_COL = std::round(H_FOV * alpha_hfov);

    std::pair<int, int> rimg{NUM_RANGE_IMG_ROW, NUM_RANGE_IMG_COL};
    return rimg;
}
struct SphericalPoint {
    float az; // azimuth
    float el; // elevation
    float r;  // radius
};
SphericalPoint cart2sph(const PointType &_cp) {
    SphericalPoint sph_point{std::atan2(_cp.y, _cp.x), std::atan2(_cp.z, std::sqrt(_cp.x * _cp.x + _cp.y * _cp.y)),
                             std::sqrt(_cp.x * _cp.x + _cp.y * _cp.y + _cp.z * _cp.z)};
    return sph_point;
}
inline float rad2deg(float radians) { return radians * 180.0 / M_PI; }
} // namespace removert