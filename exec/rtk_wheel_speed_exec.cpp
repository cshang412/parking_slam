#include "rtk_wheel_speed_exec.h"

#include "common/type_converter.h"

namespace parking_slam
{

REGISTER_EXECUTER(RTKExec)
REGISTER_EXECUTER(WheelSpeedExec)
REGISTER_EXECUTER(RTKWheelSpeedExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int RTKWheelSpeedExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    if (DataCenter::get_instance()->state_machine().get_status() != StateMachine::Status::START)
    {
        return -1;
    }

    const auto p_bundle = dynamic_cast<const ::haomo::pos::BundleData*>(input);
    if (p_bundle == nullptr)
    {
        LOG_ERROR << "input bundle" << input->get_name() << "dynamic_case error";
        return -1;
    }

    if (p_bundle->get<::haomo::hidelivery::WheelSpeedData>(FLAGS_wheelspeed_data_name) != nullptr &&
        p_bundle->get<::haomo::hidelivery::RtkData>(FLAGS_rtk_data_name) != nullptr)
    {
        auto rtk_data = p_bundle->get<::haomo::hidelivery::RtkData>(FLAGS_rtk_data_name)->data();

        // static auto last_t = rtk_data->sys_time_us();
        // auto        tt     = (rtk_data->sys_time_us() - last_t);
        // last_t = rtk_data->sys_time_us();

        auto rtk_msg = std::make_shared<parking_slam::RtkMsg>(rtk_data->sys_time_us() * 1e-6, rtk_data->sys_time_us());
        rtk_msg->rtk_status  = rtk_data->rtk_sts();
        rtk_msg->euler_angle = Eigen::Vector3d(rtk_data->angle().azimuth_angle(), rtk_data->angle().pitch_angle(),
                                               rtk_data->angle().roll_angle());

        rtk_msg->llh = Eigen::Vector3d(rtk_data->lon_lat_height().latitude(), rtk_data->lon_lat_height().longitude(),
                                       rtk_data->lon_lat_height().height());

        rtk_msg->rtk_velocity = Eigen::Vector3d(rtk_data->speed().north_speed(), rtk_data->speed().east_speed(),
                                                rtk_data->speed().earth_speed());

        rtk_msg->acceleration = Eigen::Vector3d(rtk_data->xyz_rate_acc().x_acc(), rtk_data->xyz_rate_acc().y_acc(),
                                                rtk_data->xyz_rate_acc().z_acc());

        rtk_msg->angle_rate =
            Eigen::Vector3d(rtk_data->xyz_rate_acc().x_angular_rate(), rtk_data->xyz_rate_acc().y_angular_rate(),
                            rtk_data->xyz_rate_acc().z_angular_rate());

        DataCenter::get_instance()->rtk_msg_deque.insert(rtk_msg);

        auto wheel_speed_data = p_bundle->get<::haomo::hidelivery::WheelSpeedData>(FLAGS_wheelspeed_data_name)->data();

        auto wheel_speed_msg = std::make_shared<parking_slam::WheelSpeedMsg>(wheel_speed_data->sys_time_us() * 1e-6,
                                                                             wheel_speed_data->sys_time_us());

        std::double_t rear_left_direction  = wheel_speed_data->rear_left_direction() == 1 ? 1 : -1;
        std::double_t rear_right_direction = wheel_speed_data->rear_right_direction() == 1 ? 1 : -1;

        wheel_speed_msg->vehicle_speed = 0.5 * (wheel_speed_data->rear_left() * rear_left_direction +
                                                wheel_speed_data->rear_right() * rear_right_direction);

        DataCenter::get_instance()->wheel_speed_msg_deque.insert(wheel_speed_msg);

        // predict current states
        // step 1. check new lidar state timestamp
        bool              pub_valid    = false;
        Eigen::Isometry3d T_world_body = Eigen::Isometry3d::Identity();
        Eigen::Vector3d   V_world_body = Eigen::Vector3d::Zero();

        rtk_msgs_.push_back(rtk_msg);
        wheel_speed_msgs_.push_back(wheel_speed_msg);

        while (rtk_msgs_.size() > 1600)
        {
            rtk_msgs_.pop_front();
        }

        while (wheel_speed_msgs_.size() > 1600)
        {
            wheel_speed_msgs_.pop_front();
        }

        if (DataCenter::get_instance()->lio_inited() &&
            DataCenter::get_instance()->state_machine().get_status() == StateMachine::Status::START)
        {
            static std::double_t last_lio_state_timestamp = 0U;

            std::double_t current_timestamp =
                DataCenter::get_instance()->get_current_lio_state().get_timestamp().second();

            if (current_timestamp > last_lio_state_timestamp)
            {
                current_predict_state_ = DataCenter::get_instance()->get_current_lio_state();

                while (!rtk_msgs_.empty() && !wheel_speed_msgs_.empty() &&
                       rtk_msgs_.front()->timestamp <= current_timestamp)
                {
                    rtk_msgs_.pop_front();
                    wheel_speed_msgs_.pop_front();
                }

                for (size_t i = 0; i < rtk_msgs_.size(); ++i)
                {
                    if (i > wheel_speed_msgs_.size())
                    {
                        break;
                    }

                    predict(rtk_msgs_[i], wheel_speed_msgs_[i]);

                    if (rtk_msgs_[i]->timestamp > (current_timestamp + 0.1))
                    {
                        break;
                    }
                }

                last_lio_state_timestamp = current_timestamp;
            }

            // predict(rtk_msg, wheel_speed_msg);

            std::double_t time_diff = 0.0;

            if (!rtk_msgs_.empty())
            {
                time_diff = std::fabs(rtk_msgs_.front()->timestamp - last_lio_state_timestamp);
            }

            {
                //@todo chenghe publish localization result
                DataCenter::get_instance()->set_predict_state(current_predict_state_);
                Eigen::Isometry3d T_world_imu =
                    parking_slam::TypeConverter::nav_state_to_iso3d(current_predict_state_.get_nav_state());
                T_world_body = T_world_imu * DataCenter::get_instance()->hardware_config.get_rtk_T_body();

                V_world_body = T_world_imu.linear() * current_predict_state_.get_nav_state().v();

                LOG_INFO << "publish localization result: posi: [" << T_world_body.translation().x() << " "
                         << T_world_body.translation().y() << " " << T_world_body.translation().z() << "], velocity: ["
                         << V_world_body.x() << " " << V_world_body.y() << " " << V_world_body.z()
                         << "], dt: " << time_diff;

                pub_valid = true;
            }
        }

        if(!pub_valid)
        {
            return -1;
        }

        auto avp_pose = ::haomo::pos::data_cast<haomo::hidelivery::localization::Pose>(output);
        if (avp_pose == nullptr)
        {
            LOG_ERROR << "output " << output->get_name() << "dynamic_cast error";
            return -1;
        }
        avp_pose->set_status(haomo::hidelivery::localization::Pose_StatusType_OK);
        avp_pose->set_sys_time_us(rtk_data->sys_time_us());

        const Eigen::Quaterniond vehicle_utm_rotation_q(T_world_body.linear());
        avp_pose->mutable_attitude()->set_qw(vehicle_utm_rotation_q.w());
        avp_pose->mutable_attitude()->set_qx(vehicle_utm_rotation_q.x());
        avp_pose->mutable_attitude()->set_qy(vehicle_utm_rotation_q.y());
        avp_pose->mutable_attitude()->set_qz(vehicle_utm_rotation_q.z());

        avp_pose->mutable_odom_attitude()->CopyFrom(avp_pose->attitude());

        const Eigen::Vector3d vehicle_utm_rotation_ypr = parking_slam::R2ypr(T_world_body.linear());
        avp_pose->mutable_rotation_rpy()->set_x(vehicle_utm_rotation_ypr.z());
        avp_pose->mutable_rotation_rpy()->set_y(vehicle_utm_rotation_ypr.y());
        avp_pose->mutable_rotation_rpy()->set_z(vehicle_utm_rotation_ypr.x());

        avp_pose->mutable_position()->set_x(T_world_body.translation().x());
        avp_pose->mutable_position()->set_y(T_world_body.translation().y());
        avp_pose->mutable_position()->set_z(T_world_body.translation().z());

        avp_pose->mutable_odom_position()->CopyFrom(avp_pose->position());

        avp_pose->mutable_velocity()->set_x(V_world_body.x());
        avp_pose->mutable_velocity()->set_y(V_world_body.y());
        avp_pose->mutable_velocity()->set_z(V_world_body.z());

        return 0;
    }

    return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTKWheelSpeedExec::predict(const parking_slam::RtkMsg::Ptr&        imu_msg,
                                const parking_slam::WheelSpeedMsg::Ptr& wheel_msg)
{
    std::double_t dt = parking_slam::Timestamp(imu_msg->timestamp_us).diff(current_predict_state_.get_timestamp());

    if (dt > 0.5)
    {
        // log error, only update timestamp
        current_predict_state_.set_timestamp(imu_msg->timestamp_us);

        LOG_ERROR << "dt too large!" << dt;

        return;
    }
    else if (dt < 0)
    {
        LOG_ERROR << "dt < 0 , dt: " << dt;

        return;
    }
    else if (dt < 1e-3)
    {
        dt = 1e-3;
    }
    else if (dt == 0.0)
    {
        return;
    }

    std::double_t wheel_speed = wheel_msg->vehicle_speed;

    bool is_zupt = std::fabs(wheel_speed) < 1e-3;

    auto new_state = current_predict_state_;
    new_state.set_timestamp(imu_msg->timestamp_us);

    if (!is_zupt)
    {
        const Eigen::Vector3d acc_unbias =
            imu_msg->acceleration - current_predict_state_.get_imu_bias().accelerometer();
        const Eigen::Vector3d gyr_unbias = imu_msg->angle_rate - current_predict_state_.get_imu_bias().gyroscope();
        const Eigen::Vector3d delta_angle_axis = gyr_unbias * dt;

        Eigen::Matrix3d new_rotation = current_predict_state_.get_nav_state().pose().rotation().matrix();

        if (delta_angle_axis.norm() > 1e-12)
        {
            new_rotation = current_predict_state_.get_nav_state().pose().rotation().matrix() *
                           parking_slam::delta_q(delta_angle_axis).toRotationMatrix();
        }

        // const std::double_t   dt2       = dt * dt;
        // Eigen::Vector3d new_translation = current_predict_state_.get_nav_state().pose().translation() +
        //                                   current_predict_state_.get_nav_state().v() * dt +
        //                                   0.5 *
        //                                       (current_predict_state_.get_nav_state().pose().rotation().matrix() *
        //                                       acc_unbias +
        //                                        Eigen::Vector3d{0, 0, -9.81}) *
        //                                       dt2;
        Eigen::Vector3d new_translation = current_predict_state_.get_nav_state().pose().translation() +
                                          current_predict_state_.get_nav_state().pose().rotation().matrix() *
                                              current_predict_state_.get_wheel_scale()[0] *
                                              DataCenter::get_instance()->hardware_config.get_rtk_T_body() *
                                              Eigen::Vector3d{wheel_msg->vehicle_speed, 0, 0} * dt;

        Eigen::Vector3d new_velocity = current_predict_state_.get_nav_state().v() +
                                       (current_predict_state_.get_nav_state().pose().rotation().matrix() * acc_unbias +
                                        Eigen::Vector3d{0, 0, -9.81}) *
                                           dt;

        gtsam::NavState new_nav_state(gtsam::Rot3(new_rotation), new_translation, new_velocity);
        new_state.set_nav_state(new_nav_state);
    }

    current_predict_state_ = new_state;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int RTKExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    const auto pdata = haomo::pos::data_cast<::haomo::hidelivery::RtkData>(input);

    if (pdata != nullptr)
    {
        auto rtk_data = pdata->data();
        auto rtk_msg  = std::make_shared<parking_slam::RtkMsg>(rtk_data->sys_time_us() * 1e-6, rtk_data->sys_time_us());
        rtk_msg->rtk_status  = rtk_data->rtk_sts();
        rtk_msg->euler_angle = Eigen::Vector3d(rtk_data->angle().azimuth_angle(), rtk_data->angle().pitch_angle(),
                                               rtk_data->angle().roll_angle());

        rtk_msg->llh = Eigen::Vector3d(rtk_data->lon_lat_height().latitude(), rtk_data->lon_lat_height().longitude(),
                                       rtk_data->lon_lat_height().height());

        rtk_msg->rtk_velocity = Eigen::Vector3d(rtk_data->speed().east_speed(), rtk_data->speed().north_speed(),
                                                rtk_data->speed().earth_speed());

        rtk_msg->acceleration = Eigen::Vector3d(rtk_data->xyz_rate_acc().x_acc(), rtk_data->xyz_rate_acc().y_acc(),
                                                rtk_data->xyz_rate_acc().z_acc());

        rtk_msg->angle_rate =
            Eigen::Vector3d(rtk_data->xyz_rate_acc().x_angular_rate(), rtk_data->xyz_rate_acc().y_angular_rate(),
                            rtk_data->xyz_rate_acc().z_angular_rate());

        DataCenter::get_instance()->rtk_msg_deque.insert(rtk_msg);
    }
    else
    {
        LOG_ERROR << "receive RTK data is null";
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int WheelSpeedExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    const auto pdata = haomo::pos::data_cast<::haomo::hidelivery::WheelSpeedData>(input);

    if (pdata != nullptr)
    {
        auto wheel_speed_data = pdata->data();
        auto wheel_speed_msg  = std::make_shared<parking_slam::WheelSpeedMsg>(wheel_speed_data->sys_time_us() * 1e-6,
                                                                             wheel_speed_data->sys_time_us());

        std::double_t rear_left_direction  = wheel_speed_data->rear_left_direction() == 2 ? -1 : 1;
        std::double_t rear_right_direction = wheel_speed_data->rear_right_direction() == 2 ? -1 : 1;

        wheel_speed_msg->vehicle_speed = 0.5 * (wheel_speed_data->rear_left() * rear_left_direction +
                                                wheel_speed_data->rear_right() * rear_right_direction);

        DataCenter::get_instance()->wheel_speed_msg_deque.insert(wheel_speed_msg);
    }
    else
    {
        LOG_ERROR << "receive Wheel Speed data is null";
    }

    return 0;
}

}  // namespace parking_slam
