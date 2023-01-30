#include "map_manager_exec.h"
#include "parking_slot_helper.h"
namespace parking_slam
{

REGISTER_EXECUTER(MapManagerExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MapManagerExec::MapManagerExec(const std::string& name, const std::string& type) : IExec(name, type)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapManagerExec::process_save_map()
{
    // while (!stop_)
    {
        if (!DataCenter::get_instance()->inited() || !DataCenter::get_instance()->lio_inited())
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }

        if (!DataCenter::get_instance()->save_map)
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }

        const std::string save_root_path = DataCenter::get_instance()->config.map_process_config().map_path();

        const auto key_frame_deque = DataCenter::get_instance()->get_current_key_frame_deque();
        const auto lio_states      = DataCenter::get_instance()->get_current_lio_state_deque();
        const auto predict_states  = DataCenter::get_instance()->get_predict_state_deque();

        if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
        {
            if (key_frame_deque.empty() || lio_states.empty())
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return;
            }

            if (predict_states.empty())
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return;
            }

            if (DataCenter::get_instance()->config.map_process_config().save_trajectory_ply() &&
                DataCenter::get_instance()->save_map)
            {
                parking_slam::MapManager::save_trajectory_ply(
                    lio_states, DataCenter::get_instance()->hardware_config.get_rtk_T_body(),
                    save_root_path + "/mapping_trajectory.ply");

                parking_slam::MapManager::save_trajectory_ply(
                    predict_states, DataCenter::get_instance()->hardware_config.get_rtk_T_body(),
                    save_root_path + "/mapping_predict_trajectory.ply");
            }

            if (DataCenter::get_instance()->config.map_process_config().save_cloud_ply() &&
                DataCenter::get_instance()->save_map)
            {
                parking_slam::MapManager::save_cloud_ply(key_frame_deque,
                                                         DataCenter::get_instance()->hardware_config.get_rtk_T_body(),
                                                         save_root_path + "/cloud_map.ply");
            }

            if (DataCenter::get_instance()->config.map_process_config().save_avp_map() &&
                DataCenter::get_instance()->save_map)
            {
                std::vector<haomo::hidelivery::perception::ParkingSlot3D> slot_list;
                auto slot_map = slot_map_;
                for(const auto& pair: slot_map)
                {
                    slot_list.push_back(pair.second);
                }
                
                haomo::avp_map::AVPMapMsg avp_map;
                parking_slam::MapManager::save_avp_map(
                    lio_states, key_frame_deque, slot_list,
                    DataCenter::get_instance()->hardware_config.get_rtk_T_body(),
                    DataCenter::get_instance()->boot_llh, DataCenter::get_instance()->config.map_process_config(),
                    save_root_path + "/avp_map.db", save_root_path + "/voxel_map.ply", avp_map);
                
                ParkingSlotHelper::save_slot_cloud(avp_map, 0.2, save_root_path + "/slot.ply");

                parking_slam::MapManager::save_processed_ref_line_ply(avp_map,
                                                                      save_root_path + "/processed_ref_line.ply");
            }
        }

        if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::LOCALIZATION)
        {
            if (predict_states.empty() || lio_states.empty())
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return;
            }

            if (DataCenter::get_instance()->config.map_process_config().save_trajectory_ply() &&
                DataCenter::get_instance()->save_map)
            {
                parking_slam::MapManager::save_trajectory_ply(
                    lio_states, DataCenter::get_instance()->hardware_config.get_rtk_T_body(),
                    save_root_path + "/loc_trajectory.ply");

                parking_slam::MapManager::save_trajectory_ply(
                    predict_states, DataCenter::get_instance()->hardware_config.get_rtk_T_body(),
                    save_root_path + "/loc_predict_trajectory.ply");
            }
        }

        DataCenter::get_instance()->save_map = false;
        DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::RESET);
        //每次建图结束后reset lio estimator
        LOG_WARNING << "save map: " << save_root_path;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int MapManagerExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    if (DataCenter::get_instance()->inited())
    {
        if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
        {
            localization_map_inited_ = false;
        }

        if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::LOCALIZATION)
        {
            if (!localization_map_inited_)
            {
                localization_map_inited_ = DataCenter::get_instance()->load_map();
            }
        }
    }

    if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
    {
        auto&& lastest_fpsd_vec  = ::haomo::pos::dao::latest<haomo::hidelivery::PerceptionData>(FLAGS_perception_fusion_parkspace_name, 1);
        if (!lastest_fpsd_vec.empty()) 
        {
            auto fusion_park_space = lastest_fpsd_vec.front()->ctx();
            const auto psds = fusion_park_space.parking_slots();
            for (auto i = 0 ; i < psds.fusion_parking_slots_world_size(); ++i) 
            {
                auto slot_id = psds.fusion_parking_slots_world(i).id();
                slot_map_[slot_id] = psds.fusion_parking_slots_world(i);
            }
        }
    }

    process_save_map();

    return 0;
}

}  // namespace parking_slam
