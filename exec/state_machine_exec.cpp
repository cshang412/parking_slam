#include "state_machine_exec.h"

namespace parking_slam
{

REGISTER_EXECUTER(StateMachineExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int StateMachineExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    const auto& pfsm_input = ::haomo::pos::param_server::latest<::haomo::hidelivery::PfsmOutput>(FLAGS_parking_fsm_data_name, 1);
    if (!pfsm_input.empty() && pfsm_input.front()->data() != nullptr)
    {
        auto pfsm_data = pfsm_input.front()->data();
        if(!pfsm_data->has_parking_state()                            ||
           pfsm_data->mode() != haomo::pfsm::PfsmStateEnum::AVP_STATE)
        {
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::STOP);

            return 0;
        }

        if(static_cast<int>(pfsm_data->parking_state().state()) >= static_cast<int>(haomo::pfsm::MAPPING) && 
           static_cast<int>(pfsm_data->parking_state().state()) <= static_cast<int>(haomo::pfsm::MAPPING_EXIT))
        {
            DataCenter::get_instance()->state_machine().set_mode(parking_slam::Mode::MAPPING);
            if(pfsm_data->parking_state().state() == haomo::pfsm::PfsmStateEnum::MAPPING)
            {
                if(pfsm_data->mapping_info().mapping_mode() == haomo::pfsm::AVPMappingTypeEnum::SHADOW)
                {
                    //@chenghe todo shadow mapping
                }
                DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::START);
            }

            if(pfsm_data->parking_state().state() == haomo::pfsm::PfsmStateEnum::MAPPING_COMPLETED)
            {
                if(DataCenter::get_instance()->state_machine().get_status() == StateMachine::Status::START)
                {
                    DataCenter::get_instance()->save_map = true;
                }
                
                DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::STOP);
            }

            if(pfsm_data->parking_state().state() == haomo::pfsm::PfsmStateEnum::MAPPING_EXIT)
            {
                DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::RESET);
            }
        }

        if((static_cast<int>(pfsm_data->parking_state().state()) >= static_cast<int>(haomo::pfsm::PfsmStateEnum::CRUISE_INIT) &&
            static_cast<int>(pfsm_data->parking_state().state()) <= static_cast<int>(haomo::pfsm::PfsmStateEnum::CRUISING_EXIT))  ||
           (static_cast<int>(pfsm_data->parking_state().state()) >= static_cast<int>(haomo::pfsm::PfsmStateEnum::SEARCHING)   && 
            static_cast<int>(pfsm_data->parking_state().state()) <= static_cast<int>(haomo::pfsm::PfsmStateEnum::PARKING_FAULT)))
        {
            DataCenter::get_instance()->state_machine().set_mode(parking_slam::Mode::LOCALIZATION);
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::START);

            if(pfsm_data->parking_state().state() == haomo::pfsm::PfsmStateEnum::CRUISING_EXIT ||
               pfsm_data->parking_state().state() == haomo::pfsm::PfsmStateEnum::PARKING_FAULT ||
               pfsm_data->parking_state().state() == haomo::pfsm::PfsmStateEnum::PARKING_EXIT)
            {
                DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::RESET);
            }
        }
    }

    // auto&& fusion_input = ::haomo::pos::param_server::latest<haomo::hidelivery::PerceptionData>("fusion_parkspace",1);
    // if(!fusion_input.empty())
    // {
    //     if(!fusion_input.empty() && fusion_input.front()->data() != nullptr && 
    //         fusion_input.front()->data()->has_parking_slots())
    //     {
    //         haomo::hidelivery::PerceptionData  perception_fusion;
    //         perception_fusion.CopyFrom(*fusion_input.front()->data());
    //         for(int i = 0; i < psd_vision_.local_parking_slots_size(); ++i)
    //         {
    //             std::vector<Eigen::Vector3d> pts = get_psd_points(psd_vision_.local_parking_slots(i), pose);
    //             if(pts.size() >= 2)
    //             {
    //                 draw_slot_pts(pts, Eigen::Vector3d{1.0, 1.0, 0.0}, 1.0);
    //             }            
    //         }
    //     }
    // }


    return 0;
}

}  // namespace parking_slam
