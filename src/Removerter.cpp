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

#include <glog/logging.h>
#include "Removerter.h"

namespace removert
{
    MapUpdater::MapUpdater(const std::string &config_file_path)
    {
        yconfig = YAML::LoadFile(config_file_path);
        MapUpdater::setConfig();
    }
    void MapUpdater::setRawMap(const pcl::PointCloud<PointT>::Ptr &rawmap){

    }
    void MapUpdater::run(const pcl::PointCloud<PointT>::Ptr &single_pc){

    }
    void MapUpdater::saveMap(const std::string &map_path){

    }
    void MapUpdater::setConfig(){
        
    }
} // namespace removert
