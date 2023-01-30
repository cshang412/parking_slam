/**
 * @file
 * @brief
 * @author chenghe
 * @author zhaoqiang
 * @date 2022-08-23
 */

#include <iostream>

#include "data.h"
#include "data_provider/dataset.h"
#include "lio_mapping.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace parking_slam;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    const bool        save_test_result      = true;
    const std::string config_path           = "/home/ubuntu/haomo/data/config.prototxt";
    const std::string dataset_root          = "/home/ubuntu/haomo/data/new/";
    const std::string save_result_file_name = "/home/ubuntu/haomo/data/result/";
    const int         end_data_num          = 35000;
    const LidarType   lidar_type            = LidarType::M1;  // M1 Center128

    parking_slam::ParkingSLAMConfig config;
    ;
    if (!::haomo::pos::read_proto_from_text_file(config_path.c_str(), &config))
    {
        std::cerr << "load config file error!" << std::endl;

        return -1;
    }

    Dataset dataset(0.02, lidar_type);
    dataset.load(dataset_root);
    // dataset.auto_compute_start_idx();
    dataset.set_start_idx(0);

    LIOMapping::Ptr lio_mapping_ptr =
        std::make_shared<LIOMapping>(config, dataset.get_hardware_config(), config.mode());

    for (int i = 0; i < end_data_num && !dataset.is_end(); ++i)
    {
        std::cout << "get dataset id: " << i << std::endl;

        auto imu_msg = dataset.get_next_rtk_msg();

        if (imu_msg != nullptr)
        {
            lio_mapping_ptr->add_imu_msg(imu_msg);
        }

        auto wheel_msg = dataset.get_next_wheel_msg();

        if (wheel_msg != nullptr)
        {
            lio_mapping_ptr->add_wheel_msg(wheel_msg);
        }

        auto point_cloud_msg = dataset.get_next_point_cloud_msg();

        if (point_cloud_msg != nullptr)
        {
            std::cout << i << std::endl;
            Timer timer;
            timer.Tic();
            LIOState                           state(0U);
            std::deque<parking_slam::LIOFrame> key_frame_deque;
            std::deque<parking_slam::LIOState> lio_state_deque;

            lio_mapping_ptr->add_point_cloud(point_cloud_msg, state, key_frame_deque, lio_state_deque);
            timer.Toc();
            std::cout << "total cost: " << timer.Duration() << std::endl;
        }
    }

    if (save_test_result)
    {
        lio_mapping_ptr->save(save_result_file_name);
    }

    return 0;
}
