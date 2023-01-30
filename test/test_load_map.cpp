/**
 * @file
 * @brief
 * @author chenghe
 * @author zhaoqiang
 * @date 2022-08-23
 */

#include <iostream>

#include "data.h"
#include "map_manager.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace parking_slam;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    const std::string config_path =
        "/home/ubuntu/haomo/parking_slam/modules/parking_slam/config/arch_x86/config.prototxt";

    parking_slam::ParkingSLAMConfig config;
    if (!::haomo::pos::read_proto_from_text_file(config_path.c_str(), &config))
    {
        std::cerr << "load config file error!" << std::endl;
        return -1;
    }
    
    haomo::avp_map::AVPMapMsg avp_map;
    parking_slam::LIOMap map;
    pcl::PointCloud<pcl::PointXYZRGB> color_map;
    if(parking_slam::MapManager::load_map(config.map_process_config(), avp_map, map, color_map))
    {
        std::cout << "success load map!" << std::endl;
    }
    

    return 0;
}
