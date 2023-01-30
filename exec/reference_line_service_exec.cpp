#include "reference_line_service_exec.h"

#include "common/type_converter.h"
#include "parking_slot_helper.h"
namespace parking_slam
{

REGISTER_EXECUTER(ReferenceLineServiceExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d localization_data_2_eigen(const haomo::hidelivery::localization::Pose& localization_data)
{
    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.linear() = Eigen::Quaterniond(localization_data.attitude().qw(), localization_data.attitude().qx(),
                                      localization_data.attitude().qy(), localization_data.attitude().qz())
                       .toRotationMatrix();

    iso.translation() = Eigen::Vector3d(localization_data.position().x(), localization_data.position().y(),
                                        localization_data.position().z());

    return iso;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void transfrom_localization_data(const Eigen::Isometry3d& iso_in, const Eigen::Isometry3d& T_out_in,
                                 haomo::hidelivery::localization::Pose& localization_data_out)
{
    Eigen::Isometry3d     iso_out = T_out_in * iso_in;
    Eigen::Quaterniond    q_out(iso_out.linear());
    const Eigen::Vector3d ypr = parking_slam::R2ypr(q_out.toRotationMatrix());

    localization_data_out.mutable_attitude()->set_qw(q_out.w());
    localization_data_out.mutable_attitude()->set_qx(q_out.x());
    localization_data_out.mutable_attitude()->set_qy(q_out.y());
    localization_data_out.mutable_attitude()->set_qz(q_out.z());

    localization_data_out.mutable_rotation_rpy()->set_x(ypr.z());
    localization_data_out.mutable_rotation_rpy()->set_y(ypr.y());
    localization_data_out.mutable_rotation_rpy()->set_z(ypr.x());

    localization_data_out.mutable_position()->set_x(iso_out.translation().x());
    localization_data_out.mutable_position()->set_y(iso_out.translation().y());
    localization_data_out.mutable_position()->set_z(iso_out.translation().z());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_perception_view(const haomo::hidelivery::localization::Pose& loc,
                             haomo::hidelivery::perception::PerceptionView& perception_view_out)
{
    auto raw_trajectory      = DataCenter::get_instance()->avp_map.raw_trajectory();
    auto raw_trajectory_size = raw_trajectory.reference_line_points_world_size();
    auto target_pose_iso     = localization_data_2_eigen(raw_trajectory.reference_line_points_world(raw_trajectory_size -1));
    auto current_pose_iso    = localization_data_2_eigen(loc);
    auto slot_world          = ParkingSlotHelper::mock_target_slot_world(target_pose_iso);
    auto slot_ego            = ParkingSlotHelper::transform_slot_to_ego(slot_world, current_pose_iso);

    auto slot_ego_out   = perception_view_out.mutable_parking_slots()->add_fusion_parking_slots_ego();
    auto slot_world_out = perception_view_out.mutable_parking_slots()->add_fusion_parking_slots_world();

    slot_ego_out->CopyFrom(slot_ego);
    slot_world_out->CopyFrom(slot_world);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_avp_map(const haomo::hidelivery::localization::Pose& loc,
                     haomo::avp_map::AVPMapMsg& avp_map_out)
{
    if(DataCenter::get_instance()->avp_map.has_target_parking_slot())
    {
        auto target_slot = DataCenter::get_instance()->avp_map.target_parking_slot();
        avp_map_out.mutable_target_parking_slot()->CopyFrom(target_slot);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool process_ref_line(const haomo::hidelivery::localization::Pose& loc,
                      haomo::avp_map::AVPReferenceLineMsg&         ref_line_out)
{
    Eigen::Isometry3d loc_iso      = localization_data_2_eigen(loc);
    std::double_t     min_distance = 1e8;
    std::double_t     min_angle_diff = 1e8;
    std::int32_t      min_idx      = -1;

    auto avp_ref_line = DataCenter::get_instance()->avp_map.ref_line();

    if (avp_ref_line.reference_line_points_world_size() < 1)
    {
        LOG_ERROR << "avp_ref_line is empty";
        return false;
    }

    ref_line_out.set_sys_time_us(loc.sys_time_us());

    for (int i = 0; i < avp_ref_line.reference_line_points_world_size(); ++i)
    {
        Eigen::Isometry3d iso_i      = localization_data_2_eigen(avp_ref_line.reference_line_points_world(i));
        std::double_t     trans_diff = (iso_i.translation() - loc_iso.translation()).norm();
        std::double_t     angle_diff = Eigen::AngleAxisd(iso_i.linear().inverse() * loc_iso.linear()).angle();

        if (angle_diff < 360.0 * M_PI / 180.0 && trans_diff < min_distance)
        {
            min_distance = trans_diff;
            min_idx      = i;
            min_angle_diff = angle_diff;
        }
    }

    if(min_distance > 5.0)
    {
        return false;
    }

    {
        std::double_t  length = 0;
        
        for(int i = min_idx; i > 0; --i)
        {
            Eigen::Isometry3d current_iso = localization_data_2_eigen(avp_ref_line.reference_line_points_world(i));
            Eigen::Isometry3d last_iso    = localization_data_2_eigen(avp_ref_line.reference_line_points_world(i-1));

            length += (current_iso.translation() - last_iso.translation()).norm();

            if(length >= 4.0)
            {
                min_idx = i;
                break;
            }
        }
    }

    LOG_WARNING << "current distance to ref line: " << min_distance;
    LOG_WARNING << "current angle to ref line: " << min_angle_diff/M_PI * 180.0;
    // min_idx += 1;

    if (min_idx >= 0 && min_idx < avp_ref_line.reference_line_points_world_size())
    {
        std::vector<Eigen::Vector3d> ref_line;
        std::double_t                length             = 0;
        std::double_t                distance_to_target = 0;
        Eigen::Isometry3d last_iso = localization_data_2_eigen(avp_ref_line.reference_line_points_world(min_idx));

        for (int i = min_idx; i < avp_ref_line.reference_line_points_world_size(); ++i)
        {
            auto ref_line_pose_world = ref_line_out.add_reference_line_points_world();
            auto ref_line_pose_ego   = ref_line_out.add_reference_line_points_ego();

            ref_line_pose_world->CopyFrom(avp_ref_line.reference_line_points_world(i));
            Eigen::Isometry3d current_iso = localization_data_2_eigen(*ref_line_pose_world);

            // transfrom world to ego;
            transfrom_localization_data(current_iso, loc_iso.inverse(), *ref_line_pose_ego);

            distance_to_target += (current_iso.translation() - last_iso.translation()).norm();

            if (length <= 50)
            {
                length += (current_iso.translation() - last_iso.translation()).norm();

                ref_line.push_back(current_iso.translation());
            }
            else
            {
                break;
            }

            last_iso = current_iso;
        }

        DataCenter::get_instance()->set_ref_line(ref_line);

        ref_line_out.set_length(length);
        ref_line_out.set_distance_to_target(distance_to_target);

        return true;
    }

    ref_line_out.set_length(0.0);

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ReferenceLineServiceExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    if (!DataCenter::get_instance()->inited() || !DataCenter::get_instance()->lio_inited())
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return -1;
    }

    if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::LOCALIZATION)
    {
        const auto& pfsm_input = ::haomo::pos::param_server::latest<::haomo::hidelivery::PfsmOutput>(FLAGS_parking_fsm_data_name, 1);
        if(pfsm_input.empty() || pfsm_input.front()->data() == nullptr)
        {
            return -1;
        }
        
        auto pfsm_data = pfsm_input.front()->data();
        if(!pfsm_data->has_parking_state()                            ||
            pfsm_data->mode() != haomo::pfsm::PfsmStateEnum::AVP_STATE)
        {
            return -1;
        }


        if(static_cast<int>(pfsm_data->parking_state().state()) == static_cast<int>(haomo::pfsm::CRUISING)     ||
          (static_cast<int>(pfsm_data->parking_state().state()) >= static_cast<int>(haomo::pfsm::SEARCHING) && 
           static_cast<int>(pfsm_data->parking_state().state()) <= static_cast<int>(haomo::pfsm::PARKING_SUSPEND)))
        {
            const auto& loc_input    = ::haomo::pos::param_server::latest<haomo::hidelivery::LocalizationData>(FLAGS_rtk_loc_data_name);
            //auto        ref_line_out = ::haomo::pos::data_cast<haomo::avp_map::AVPReferenceLineMsg>(output);
            auto ref_line_out        = get_write_ptr<haomo::hidelivery::ParkingRoutingData>("avp_refline");
            // auto perception_view_out = get_write_ptr<haomo::hidelivery::PerceptionData>("perception_view");
            auto avp_map_out         = get_write_ptr<haomo::hidelivery::AVPMapData>("avp_map");

            if (ref_line_out == nullptr)
            {
                LOG_ERROR << "output: avp_refline ,dynamic_cast error";
                return -1;
            }

            if (!DataCenter::get_instance()->avp_map.has_ref_line())
            {
                LOG_ERROR << "no ref line data in map";
                return -1;
            }

            if (loc_input.empty())
            {
                LOG_ERROR << "loc input empty";
                return -1;
            }

            haomo::hidelivery::localization::Pose loc;
            loc.CopyFrom(*loc_input.front()->data());

            if(process_ref_line(loc, *ref_line_out->data()))
            {
                ref_line_out->data()->set_sys_time_us(loc.sys_time_us());
                notify_by_output(ref_line_out);
            }
            process_avp_map(loc, *avp_map_out->data());
            avp_map_out->data()->set_sys_time_us(loc.sys_time_us());
            notify_by_output(avp_map_out);

            return 0;
        }
    
    }

    return -1;
}

}  // namespace parking_slam
