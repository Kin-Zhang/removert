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

#include <filesystem>
#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include "Removerter.h"

namespace removert {

MapUpdater::MapUpdater(const std::string &config_file_path) {
    yconfig = YAML::LoadFile(config_file_path);
    MapUpdater::setConfig();
    LOG_IF(INFO, cfg_.verbose_) << "Config file loaded: " << std::filesystem::canonical(config_file_path);

    // init the map
    map_arranged_.reset(new pcl::PointCloud<PointType>());
    map_cleaned_.reset(new pcl::PointCloud<PointType>());
    map_dynamic_.reset(new pcl::PointCloud<PointType>());
}
void VoxelPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<PointType>::Ptr& cloud_voxelized, const double voxel_size) {
    if(voxel_size < 0.05) {
        *cloud_voxelized = *cloud;
        LOG_IF(WARNING, false) << "Voxel size is too small, no need to voxel grid filter!";
        return;
    }
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_voxelized);
}
void MapUpdater::setRawMap(const pcl::PointCloud<PointType>::Ptr &raw_map) {
    // copy raw map to map_arranged
    timing.start("0. Read RawMap  ");
    map_arranged_.reset(new pcl::PointCloud<PointType>());
    VoxelPointCloud(raw_map, map_arranged_, cfg_.downsample_voxel_size_);
    // pcl::copyPointCloud(*raw_map, *map_arranged_);
    timing.stop("0. Read RawMap  ");
}

void MapUpdater::run(const pcl::PointCloud<PointType>::Ptr &single_pc, float _rm_res) {
    // read pose in VIEWPOINT Field in pcd
    x_curr = single_pc->sensor_origin_[0];
    y_curr = single_pc->sensor_origin_[1];
    z_curr = single_pc->sensor_origin_[2];
    pcl::PointCloud<PointType>::Ptr filter_pc(new pcl::PointCloud<PointType>());
    VoxelPointCloud(single_pc, filter_pc, cfg_.downsample_voxel_size_);
    // filter spec (i.e., a shape of the range image)
    curr_res_alpha_ = _rm_res;
    std::pair<int, int> rimg_shape = resetRimgSize(cfg_.kFOV, _rm_res);
    float deg_per_pixel = 1.0 / _rm_res;

    // 1. convert to range image
    timing.start("1. Covert DepthI");
    cv::Mat scan_rimg = scan2RangeImg(filter_pc, cfg_.kFOV, rimg_shape);
    auto [map_rimg, map_rimg_ptidx] = map2RangeImg(map_arranged_, cfg_.kFOV, rimg_shape);
    timing.stop("1. Covert DepthI");

    // debug to see the img.
    // cv::Mat dst;
    // cv::normalize(map_rimg, dst, 0, 1, cv::NORM_MINMAX);
    // cv::imshow("test", dst);
    // cv::waitKey(0);

    // 2. compare range image
    timing.start("2. Compare Image");
    const int kNumRimgRow = rimg_shape.first;
    const int kNumRimgCol = rimg_shape.second;
    // float matrix, save range value
    cv::Mat diff_rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(0.0));
    cv::absdiff(scan_rimg, map_rimg, diff_rimg);
    timing.stop("2. Compare Image");

    // 3. parse dynamic points
    timing.start("3. Get DynamicId");
    std::vector<int> this_scan_dynamic_point_indexes =
        calcDescrepancyAndParseDynamicPointIdx(scan_rimg, diff_rimg, map_rimg_ptidx);
    dynamic_point_indexes.insert(dynamic_point_indexes.end(), this_scan_dynamic_point_indexes.begin(),
                                 this_scan_dynamic_point_indexes.end());
    timing.stop("3. Get DynamicId");
}

void MapUpdater::parseDynamicIdx2PointCloud() {
    // remove repeated indexes
    std::set<int> dynamic_point_indexes_set(dynamic_point_indexes.begin(), dynamic_point_indexes.end());
    std::vector<int> dynamic_point_indexes_unique(dynamic_point_indexes_set.begin(), dynamic_point_indexes_set.end());
    LOG_IF(INFO, cfg_.verbose_) << "Num of dynamic points: " << dynamic_point_indexes_unique.size() << " / "
                                << map_arranged_->size();

    // get pointcloud
    GetPointcloudUsingPtIdx(dynamic_point_indexes_unique, map_arranged_, map_cleaned_, true);
    GetPointcloudUsingPtIdx(dynamic_point_indexes_unique, map_arranged_, map_dynamic_, false);

    if (cfg_.replace_intensity) {
        for (int i = 0; i < map_dynamic_->size(); i++) {
            map_dynamic_->points[i].intensity = 1;
        }
        for (int i = 0; i < map_cleaned_->size(); i++) {
            map_cleaned_->points[i].intensity = 0;
        }
        LOG(INFO) << "\nIntensity replaced... \n";
    }
}

void MapUpdater::saveMap(const std::string &folder_path) {
    // save map_static_estimate_
    if (map_cleaned_->size() == 0) {
        LOG(WARNING) << "map_cleaned_ is empty, no map is saved";
        return;
    }
    if (map_dynamic_->size() > 0 && cfg_.replace_intensity) {
        pcl::io::savePCDFileBinary(folder_path + "/removert_output_whole.pcd", *map_dynamic_ + *map_cleaned_);
    }
    pcl::io::savePCDFileBinary(folder_path + "/removert_output.pcd", *map_cleaned_);

    LOG(INFO) << "Map saved to this folder: " << std::filesystem::canonical(folder_path);
}

void MapUpdater::setConfig() {
    cfg_.remove_resolution_list_ = yconfig["removert"]["remove_resolution_list"].as<std::vector<double>>();

    float kVFOV = yconfig["removert"]["sequence_vfov"].as<float>();
    float kHFOV = yconfig["removert"]["sequence_hfov"].as<float>();
    cfg_.kFOV = std::pair<float, float>(kVFOV, kHFOV);

    cfg_.kNumOmpCores = yconfig["removert"]["num_omp_cores"].as<int>();
    cfg_.replace_intensity = yconfig["removert"]["replace_intensity"].as<bool>();

    cfg_.downsample_voxel_size_ = yconfig["removert"]["downsample_voxel_size"].as<float>();
}

void MapUpdater::GetPointcloudUsingPtIdx(std::vector<int> &_point_indexes, pcl::PointCloud<PointType>::Ptr &_pointcloud,
                                         pcl::PointCloud<PointType>::Ptr &_pointcloud_out, bool _is_negative) {
    // extractor
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
    extractor.setInputCloud(_pointcloud);
    extractor.setIndices(index_ptr);
    // If set to true, you can extract point clouds outside the specified index
    extractor.setNegative(_is_negative);
    _pointcloud_out->clear();
    extractor.filter(*_pointcloud_out);
}

std::vector<int> MapUpdater::calcDescrepancyAndParseDynamicPointIdx(const cv::Mat &_scan_rimg,
                                                                    const cv::Mat &_diff_rimg,
                                                                    const cv::Mat &_map_rimg_ptidx) {
    int num_dyna_points{0}; // TODO: tracking the number of dynamic-assigned points and decide when to stop removing
                            // (currently just fixed iteration e.g., [2.5, 2.0, 1.5])

    std::vector<int> dynamic_point_indexes;
    for (int row_idx = 0; row_idx < _diff_rimg.rows; row_idx++) {
        for (int col_idx = 0; col_idx < _diff_rimg.cols; col_idx++) {
            float this_diff = _diff_rimg.at<float>(row_idx, col_idx);
            float this_range = _scan_rimg.at<float>(row_idx, col_idx);

            float adaptive_coeff =
                0.05; // meter, // i.e., if 4m apart point, it should be 0.4m be diff (nearer) wrt the query
            float adaptive_dynamic_descrepancy_threshold =
                adaptive_coeff * this_range; // adaptive descrepancy threshold
            // float adaptive_dynamic_descrepancy_threshold = 0.1;

            if (this_diff < kValidDiffUpperBound // exclude no-point pixels either on scan img or map img (100 is
                                                 // roughly 100 meter)
                && this_diff > adaptive_dynamic_descrepancy_threshold /* dynamic */) { // dynamic
                int this_point_idx_in_global_map = _map_rimg_ptidx.at<int>(row_idx, col_idx);
                dynamic_point_indexes.emplace_back(this_point_idx_in_global_map);
                // num_dyna_points++; // TODO
            }
        }
    }

    return dynamic_point_indexes;
} // calcDescrepancyAndParseDynamicPointIdx

cv::Mat
MapUpdater::scan2RangeImg(const pcl::PointCloud<PointType>::Ptr &_scan,
                          const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                          const std::pair<int, int> _rimg_size) {
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;

    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;

    // @ range image initizliation
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1,
                           cv::Scalar::all(kFlagNoPOINT)); // float matrix

    // @ points to range img
    int num_points = _scan->points.size();
#pragma omp parallel for num_threads(cfg_.kNumOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx) {
        PointType this_point = _scan->points[pt_idx];
        this_point.x = this_point.x - x_curr;
        this_point.y = this_point.y - y_curr;
        this_point.z = this_point.z - z_curr;

        SphericalPoint sph_point = cart2sph(this_point);
        int lower_bound_row_idx{0};
        int lower_bound_col_idx{0};
        int upper_bound_row_idx{kNumRimgRow - 1};
        int upper_bound_col_idx{kNumRimgCol - 1};
        // clang-format off
        int pixel_idx_row = int(std::min(std::max(std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) / (kVFOV - float(0.0)))), float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(std::round(kNumRimgCol *((rad2deg(sph_point.az) + (kHFOV / float(2.0))) /(kHFOV - float(0.0)))),float(lower_bound_col_idx)),float(upper_bound_col_idx)));
        // clang-format on
        float curr_range = sph_point.r;
        if (curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col)) {
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
        }
    }
    return rimg;
} // scan2RangeImg

std::pair<cv::Mat, cv::Mat>
MapUpdater::map2RangeImg(const pcl::PointCloud<PointType>::Ptr &_scan,
                         const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                         const std::pair<int, int> _rimg_size) {
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;

    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;

    // @ range image initizliation
    cv::Mat rimg =
        cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(kFlagNoPOINT)); // float matrix, save range value
    cv::Mat rimg_ptidx =
        cv::Mat(kNumRimgRow, kNumRimgCol, CV_32SC1, cv::Scalar::all(0)); // int matrix, save point (of global map) index

    // @ points to range img
    int num_points = _scan->points.size();
#pragma omp parallel for num_threads(cfg_.kNumOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx) {
        PointType this_point = _scan->points[pt_idx];
        this_point.x = this_point.x - x_curr;
        this_point.y = this_point.y - y_curr;
        this_point.z = this_point.z - z_curr;
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks.
        int lower_bound_row_idx{0};
        int lower_bound_col_idx{0};
        int upper_bound_row_idx{kNumRimgRow - 1};
        int upper_bound_col_idx{kNumRimgCol - 1};
        int pixel_idx_row =
            int(std::min(std::max(std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) /
                                                                    (kVFOV - float(0.0)))),
                                  float(lower_bound_row_idx)),
                         float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(
            std::max(std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV / float(2.0))) / (kHFOV - float(0.0)))),
                     float(lower_bound_col_idx)),
            float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        if (curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col)) {
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
            rimg_ptidx.at<int>(pixel_idx_row, pixel_idx_col) = pt_idx;
        }
    }

    return std::pair<cv::Mat, cv::Mat>(rimg, rimg_ptidx);
} // map2RangeImg

} // namespace removert
