#include "pose_manager_exec.h"

namespace parking_slam
{

REGISTER_EXECUTER(PoseManagerExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int PoseManagerExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    const auto& rtk_loc_input =
        ::haomo::pos::param_server::latest<haomo::hidelivery::LocalizationData>(FLAGS_rtk_loc_data_name);
    const auto& avp_loc_input = ::haomo::pos::param_server::latest<haomo::hidelivery::LocalizationData>("avp_loc");
    auto        pose_out      = ::haomo::pos::data_cast<haomo::hidelivery::localization::Pose>(output);

    if (pose_out == nullptr)
    {
        LOG_ERROR << "output " << output->get_name() << "dynamic_cast error";
        return -1;
    }

    if (rtk_loc_input.empty() && avp_loc_input.empty())
    {
        LOG_ERROR << "get rtk_loc & avp_loc failed!";
        pose_out->set_status(haomo::hidelivery::localization::Pose_StatusType_NOT_WORK);
        return -1;
    }

    if (rtk_loc_input.empty() || (!avp_loc_input.empty() && avp_loc_input.front()->data()->status() ==
                                                                haomo::hidelivery::localization::Pose_StatusType_OK))
    {
        pose_out->CopyFrom(*avp_loc_input.front()->data());
        pose_out->set_sys_time_us(avp_loc_input.front()->data()->sys_time_us());
        pose_out->set_map_version("avp_loc");

        pose_out->mutable_odom_attitude()->set_qw(rtk_loc_input.front()->data()->odom_attitude().qw());
        pose_out->mutable_odom_attitude()->set_qx(rtk_loc_input.front()->data()->odom_attitude().qx());
        pose_out->mutable_odom_attitude()->set_qy(rtk_loc_input.front()->data()->odom_attitude().qy());
        pose_out->mutable_odom_attitude()->set_qz(rtk_loc_input.front()->data()->odom_attitude().qz());

        pose_out->mutable_odom_position()->set_x(rtk_loc_input.front()->data()->odom_position().x());
        pose_out->mutable_odom_position()->set_y(rtk_loc_input.front()->data()->odom_position().y());
        pose_out->mutable_odom_position()->set_z(rtk_loc_input.front()->data()->odom_position().z());

        pose_out->mutable_odom_velocity()->set_x(rtk_loc_input.front()->data()->odom_velocity().x());
        pose_out->mutable_odom_velocity()->set_y(rtk_loc_input.front()->data()->odom_velocity().y());
        pose_out->mutable_odom_velocity()->set_z(rtk_loc_input.front()->data()->odom_velocity().z());
    }
    else if (!rtk_loc_input.empty())
    {
        pose_out->CopyFrom(*rtk_loc_input.front()->data());
        pose_out->set_map_version("egomotion");

        pose_out->set_sys_time_us(rtk_loc_input.front()->data()->sys_time_us());

        Eigen::Quaterniond q(
            rtk_loc_input.front()->data()->odom_attitude().qw(), rtk_loc_input.front()->data()->odom_attitude().qx(),
            rtk_loc_input.front()->data()->odom_attitude().qy(), rtk_loc_input.front()->data()->odom_attitude().qz());

        const Eigen::Vector3d vehicle_utm_rotation_ypr = parking_slam::R2ypr(q.toRotationMatrix());

        pose_out->mutable_attitude()->set_qw(rtk_loc_input.front()->data()->odom_attitude().qw());
        pose_out->mutable_attitude()->set_qx(rtk_loc_input.front()->data()->odom_attitude().qx());
        pose_out->mutable_attitude()->set_qy(rtk_loc_input.front()->data()->odom_attitude().qy());
        pose_out->mutable_attitude()->set_qz(rtk_loc_input.front()->data()->odom_attitude().qz());

        pose_out->mutable_rotation_rpy()->set_x(vehicle_utm_rotation_ypr.z());
        pose_out->mutable_rotation_rpy()->set_y(vehicle_utm_rotation_ypr.y());
        pose_out->mutable_rotation_rpy()->set_z(vehicle_utm_rotation_ypr.x());

        pose_out->mutable_position()->set_x(rtk_loc_input.front()->data()->odom_position().x());
        pose_out->mutable_position()->set_y(rtk_loc_input.front()->data()->odom_position().y());
        pose_out->mutable_position()->set_z(rtk_loc_input.front()->data()->odom_position().z());

        pose_out->mutable_velocity()->set_x(rtk_loc_input.front()->data()->odom_velocity().x());
        pose_out->mutable_velocity()->set_y(rtk_loc_input.front()->data()->odom_velocity().y());
        pose_out->mutable_velocity()->set_z(rtk_loc_input.front()->data()->odom_velocity().z());
    }

    return 0;
}

}  // namespace parking_slam
