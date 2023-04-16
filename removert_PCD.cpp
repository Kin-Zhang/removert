/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-06 16:55
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "Removerter.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc < 3) {
        LOG(ERROR) << "Usage: ./removert_run [pcd_folder] [config_file]";
        return 0;
    }
    std::string pcd_parent = argv[1]; // we assume that rawmap is in pcd_parent;
    std::string config_file = argv[2];
    int cnt = 1, run_max = 1;
    // check if the config_file exists
    if (!std::filesystem::exists(config_file)) {
        LOG(ERROR) << "Config file does not exist: " << config_file;
        return 0;
    }
    removert::MapUpdater map_updater(config_file);

    // load raw map
    std::string rawmap_path = pcd_parent + "/raw_map.pcd";
    pcl::PointCloud<PointType>::Ptr rawmap(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile<PointType>(rawmap_path, *rawmap);
    LOG(INFO) << "Raw map loaded, size: " << rawmap->size();

    map_updater.setRawMap(rawmap);

    std::vector<std::string> filenames;
    for (const auto &entry : std::filesystem::directory_iterator(std::filesystem::path(pcd_parent) / "pcd")) {
        filenames.push_back(entry.path().string());
    }

    // sort the filenames
    std::sort(filenames.begin(), filenames.end());
    int total = filenames.size();
    if (argc > 3) {
        run_max = std::stoi(argv[3]);
        if (run_max == -1) {
            LOG(INFO) << "We will run all the frame in sequence, the total "
                         "number is: "
                      << total;
            run_max = total + 1;
        }
    }
    LOG(INFO) << "rm res len: " << map_updater.cfg_.remove_resolution_list_.size();
    for (float _rm_res : map_updater.cfg_.remove_resolution_list_) {
        for (const auto &filename : filenames) {
            if (cnt > 1) {
                // LOG(INFO) << "IN";
                std::ostringstream log_msg;
                log_msg << "(" << cnt << "/" << run_max << ") Processing: " << filename << " Time Cost: "
                        << map_updater.timing.lastSeconds("1. Covert DepthI") +
                               map_updater.timing.lastSeconds("2. Compare Image") +
                               map_updater.timing.lastSeconds("3. Get DynamicId")
                        << "s";
                std::string spaces(10, ' ');
                log_msg << spaces;
                std::cout << "\r" << log_msg.str() << std::flush;
            }

            if (filename.find(".pcd") == std::string::npos)
                continue;
            pcl::PointCloud<PointType>::Ptr pcd(new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile<PointType>(filename, *pcd);
            map_updater.run(pcd, _rm_res);
            cnt++;
            if (cnt > run_max)
                break;
        }
        LOG(INFO) << "\nFinished res: " << _rm_res;
        cnt = 1; // refresh
        // break;
    }
    map_updater.timing.start("4. Parse Idx2Pcd");
    map_updater.parseDynamicIdx2PointCloud();
    map_updater.timing.stop("4. Parse Idx2Pcd");

    map_updater.timing.start("5. Write        ");
    map_updater.saveMap(pcd_parent);
    map_updater.timing.stop("5. Write        ");

    // set print color
    map_updater.timing.setColor("0. Read RawMap  ", ufo::Timing::boldYellowColor());
    map_updater.timing.setColor("1. Covert DepthI", ufo::Timing::boldCyanColor());
    map_updater.timing.setColor("2. Compare Image", ufo::Timing::boldMagentaColor());
    map_updater.timing.setColor("3. Get DynamicId", ufo::Timing::boldGreenColor());
    map_updater.timing.setColor("4. Parse Idx2Pcd", ufo::Timing::boldRedColor());
    map_updater.timing.setColor("5. Write        ", ufo::Timing::boldRedColor());
    printf("\nRemovert Timings:\n");
    printf("\t Component\t\tTotal\tLast\tMean\tStDev\t Min\t Max\t Steps\n");
    for (auto const &tag : map_updater.timing.tags()) {
        printf("\t%s%s\t%5.2f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%6lu%s\n", map_updater.timing.color(tag).c_str(),
               tag.c_str(), map_updater.timing.totalSeconds(tag), map_updater.timing.lastSeconds(tag),
               map_updater.timing.meanSeconds(tag), map_updater.timing.stdSeconds(tag),
               map_updater.timing.minSeconds(tag), map_updater.timing.maxSeconds(tag),
               map_updater.timing.numSamples(tag), ufo::Timing::resetColor());
    }

    return 0;
}