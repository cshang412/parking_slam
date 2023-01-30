#include "lidar_receiver_exec.h"

namespace parking_slam
{

REGISTER_EXECUTER(LIDARReceiveExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int LIDARReceiveExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    if (DataCenter::get_instance()->state_machine().get_status() != StateMachine::Status::START)
    {
        return 0;
    }

    const auto p_bundle = dynamic_cast<const ::haomo::pos::BundleData*>(input);
    if (p_bundle == nullptr)
    {
        LOG_ERROR << "input bundle" << input->get_name() << "dynamic_case error";
        return -1;
    }

    auto left_point_cloud_msg_ptr  = parse_point_cloud_msg(p_bundle, FLAGS_left_M1_lidar_data_name);
    auto right_point_cloud_msg_ptr = parse_point_cloud_msg(p_bundle, FLAGS_right_M1_lidar_data_name);

    if (right_point_cloud_msg_ptr != nullptr && left_point_cloud_msg_ptr != nullptr)
    {
        auto lidar_merge_point_cloud_msg = std::make_shared<parking_slam::MultiSyncedPointCloudMsg>(
            right_point_cloud_msg_ptr->timestamp, right_point_cloud_msg_ptr->timestamp_us);

        lidar_merge_point_cloud_msg->right_m1_lidar = right_point_cloud_msg_ptr;
        lidar_merge_point_cloud_msg->left_m1_lidar  = left_point_cloud_msg_ptr;
        // std::double_t time_diff = std::abs(left_point_cloud_msg_ptr->timestamp -
        // right_point_cloud_msg_ptr->timestamp); if(time_diff > 0.015)
        // {
        //     lidar_merge_point_cloud_msg->left_m1_lidar->cloud_ptr->clear();
        // }

        DataCenter::get_instance()->point_cloud_msg_deque.insert(lidar_merge_point_cloud_msg);
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
parking_slam::PointCloudMsg::Ptr LIDARReceiveExec::parse_point_cloud_msg(const ::haomo::pos::BundleData* p_bundle,
                                                                        const std::string&              msg_name) const
{
    if (p_bundle->get<::haomo::hidelivery::PointCloudData>(msg_name) != nullptr)
    {
        auto lidar_data = p_bundle->get<::haomo::hidelivery::PointCloudData>(msg_name)->data();

        parking_slam::LidarName lidar_name = parking_slam::LidarName::Left_M1;

        if (msg_name == FLAGS_right_M1_lidar_data_name)
        {
            lidar_name = parking_slam::LidarName::Right_M1;
        }

        auto point_cloud_msg_ptr = std::make_shared<parking_slam::PointCloudMsg>(
            lidar_name, lidar_data->sys_time_us() * 1e-6, lidar_data->sys_time_us());
        point_cloud_msg_ptr->cloud_ptr->reserve(lidar_data->point_size());
        for (int i = 0; i < lidar_data->point_size(); i++)
        {
            if (std::isnan(lidar_data->point(i).x()) || std::isnan(lidar_data->point(i).y()) ||
                std::isnan(lidar_data->point(i).z()))
            {
                continue;
            }
            PointXYZTR pt;
            pt.x         = lidar_data->point(i).x();
            pt.y         = lidar_data->point(i).y();
            pt.z         = lidar_data->point(i).z();
            pt.intensity = lidar_data->point(i).intensity();
            pt.timestamp = lidar_data->point(i).timestamp();
            pt.ring      = lidar_data->point(i).color();

            point_cloud_msg_ptr->cloud_ptr->push_back(pt);
        }

        return point_cloud_msg_ptr;
    }
    else
    {
        LOG_ERROR << "receive LIDAR data is null ";
    }

    return nullptr;
}

}  // namespace parking_slam
