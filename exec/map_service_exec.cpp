#include "map_service_exec.h"

#include "common/type_converter.h"

namespace parking_slam
{

REGISTER_EXECUTER(MapServiceExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_avp_map(const std::deque<LIOState>& lio_states, const std::deque<parking_slam::LIOFrame>& key_frame_deque,
                     const Eigen::Isometry3d& T_imu_body, haomo::hidelivery::PointCloudDataPb* pb_data)
{
    if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
    {
        parking_slam::MultiplicativeGaussianVoxelMap multi_voxel_map(2.0);
        parking_slam::GaussianVoxelMap               gaussian_voxel_map(1.5);

        for (size_t i = 1; i < key_frame_deque.size(); ++i)
        {
            const auto&       frame        = key_frame_deque[i];
            Eigen::Isometry3d T_world_imu  = TypeConverter::nav_state_to_iso3d(frame.state.get_nav_state());
            Eigen::Isometry3d T_world_body = T_world_imu * T_imu_body;

            gaussian_voxel_map.merge(*frame.gaussian_voxel_map_ptr, T_world_body, false);
        }

        multi_voxel_map.create_voxel_map(gaussian_voxel_map);

        if (multi_voxel_map.get_voxel_map().size() == 0)
        {
            return;
        }

        for (const auto& pair : multi_voxel_map.get_voxel_map())
        {
            auto* _point = pb_data->add_point();

            if (pair.second->get_mean().z() > 1.5)
            {
                continue;
            }

            _point->set_x(pair.second->get_mean().x());
            _point->set_y(pair.second->get_mean().y());
            _point->set_z(pair.second->get_mean().z());
        }

        pb_data->set_sys_time_us(lio_states.front().get_timestamp().get());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int MapServiceExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    if (!DataCenter::get_instance()->inited() || !DataCenter::get_instance()->lio_inited())
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return -1;
    }

    auto avp_map_out = ::haomo::pos::data_cast<haomo::hidelivery::PointCloudDataPb>(output);

    if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
    {
        const auto key_frame_deque = DataCenter::get_instance()->get_current_key_frame_deque();
        const auto lio_states      = DataCenter::get_instance()->get_current_lio_state_deque();

        if (key_frame_deque.empty() || lio_states.empty())
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));

            return -1;
        }

        const auto T_imu_body = DataCenter::get_instance()->hardware_config.get_rtk_T_body();

        if (avp_map_out == nullptr)
        {
            LOG_ERROR << "output: " << output->get_name() << " ,dynamic_cast error";
            return -1;
        }

        process_avp_map(lio_states, key_frame_deque, T_imu_body, avp_map_out);

        return 0;
    }

    return -1;
}

}  // namespace parking_slam
