#include "parking_slam_fusion_exec.h"

namespace parking_slam
{

REGISTER_EXECUTER(ParkingSLAMFusionExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParkingSLAMFusionExec::reset_data()
{
    lio_estimator_ptr_->reset_estimator();
    lio_estimator_ptr_.reset();
    DataCenter::get_instance()->reset_lio_data();
    DataCenter::get_instance()->clear_sensor_data();
    inited_ = false;
    feed_   = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParkingSLAMFusionExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    // while (!stop_)
    {
        if (!DataCenter::get_instance()->inited())
        {
            DataCenter::get_instance()->load_hardware_config();
        }

        if (DataCenter::get_instance()->state_machine().get_status() == StateMachine::Status::RESET &&
            lio_estimator_ptr_ != nullptr)
        {
            reset_data();
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::STOP);
        }

        if (lio_estimator_ptr_ != nullptr &&
            lio_estimator_ptr_->get_mode() != DataCenter::get_instance()->state_machine().get_mode())
        {
            reset_data();
        }

        if (!inited_)
        {
            if (!create_lio_estimator_ptr())
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // continue;
                return 0;
            }
        }

        if (DataCenter::get_instance()->state_machine().get_status() == StateMachine::Status::START &&
            !DataCenter::get_instance()->point_cloud_msg_deque.empty() && lio_estimator_ptr_ != nullptr)
        {
            // step1 check newest lidar timestamp
            // @todo chenghe 如果rtk 或者 轮速时间戳都比当前lidar 时间戳大怎么

            Measurement input_measurement;
            input_measurement.lidar_msg = DataCenter::get_instance()->point_cloud_msg_deque.front();

            static std::double_t last_lidar_timestamp = 0.0;

            if (last_lidar_timestamp == 0.0)
            {
                last_lidar_timestamp = input_measurement.lidar_msg->timestamp;

                DataCenter::get_instance()->point_cloud_msg_deque.pop_front();

                return 0;

                // std::this_thread::sleep_for(std::chrono::milliseconds(10));

                // continue;
            }

            // step2 sync lidar data & rtk data && wheel speed data
            for (size_t i = 0; i < DataCenter::get_instance()->rtk_msg_deque.size(); ++i)
            {
                auto rtk_msg = DataCenter::get_instance()->rtk_msg_deque[i];
                if (rtk_msg->timestamp <= input_measurement.lidar_msg->timestamp &&
                    rtk_msg->timestamp >= last_lidar_timestamp)
                {
                    input_measurement.rtk_msgs.push_back(rtk_msg);
                    input_measurement.msg_queue.push_back(rtk_msg);
                }
                else if (rtk_msg->timestamp > input_measurement.lidar_msg->timestamp)
                {
                    break;
                }
            }

            for (size_t i = 0; i < DataCenter::get_instance()->wheel_speed_msg_deque.size(); ++i)
            {
                auto wheel_speed_msg = DataCenter::get_instance()->wheel_speed_msg_deque[i];
                if (wheel_speed_msg->timestamp <= input_measurement.lidar_msg->timestamp &&
                    wheel_speed_msg->timestamp >= last_lidar_timestamp)
                {
                    input_measurement.wheel_speed_msgs.push_back(wheel_speed_msg);
                    input_measurement.msg_queue.push_back(wheel_speed_msg);
                }
                else if (wheel_speed_msg->timestamp > input_measurement.lidar_msg->timestamp)
                {
                    break;
                }
            }

            // @todo chenghe 考虑时间戳跳变时，sync逻辑
            if (input_measurement.rtk_msgs.size() >= 3 && input_measurement.wheel_speed_msgs.size() >= 3)
            {
                // std::cout<<"*******************************************************"<<std::endl;
                // std::cout<<"sync rtk msgs size: "<<rtk_msgs.size()<<std::endl;
                // std::cout<<"sync wheel speed msgs size: "<<wheel_speed_msgs.size()<<std::endl;
                // std::cout<<"rtk start:           "<<rtk_msgs.front()->timestamp_us<<std::endl;
                // std::cout<<"rtk end:             "<<rtk_msgs.back()->timestamp_us<<std::endl;

                // std::cout<<"wheel speed start:   "<<wheel_speed_msgs.front()->timestamp_us<<std::endl;
                // std::cout<<"wheel speed end:     "<<wheel_speed_msgs.back()->timestamp_us<<std::endl;

                // std::cout<<"lidar start:         "<<static_cast<std::uint64_t>(last_lidar_timestamp*1e6)<<std::endl;
                // std::cout<<"lidar end:           "  <<lidar_msg->timestamp_us<<std::endl;

                input_measurements_.push_back(input_measurement);

                while (!DataCenter::get_instance()->wheel_speed_msg_deque.empty() &&
                       DataCenter::get_instance()->wheel_speed_msg_deque.front()->timestamp < last_lidar_timestamp)
                {
                    DataCenter::get_instance()->wheel_speed_msg_deque.pop_front();
                }

                while (!DataCenter::get_instance()->rtk_msg_deque.empty() &&
                       DataCenter::get_instance()->rtk_msg_deque.front()->timestamp < last_lidar_timestamp)
                {
                    DataCenter::get_instance()->rtk_msg_deque.pop_front();
                }

                last_lidar_timestamp = input_measurement.lidar_msg->timestamp;

                while (!DataCenter::get_instance()->point_cloud_msg_deque.empty() &&
                       DataCenter::get_instance()->point_cloud_msg_deque.front()->timestamp <= last_lidar_timestamp)
                {
                    DataCenter::get_instance()->point_cloud_msg_deque.pop_front();
                }
            }
            else
            {
                LOG_WARNING << "sync failed! lidar msg too early!";

                if (!DataCenter::get_instance()->point_cloud_msg_deque.empty())
                {
                    DataCenter::get_instance()->point_cloud_msg_deque.pop_front();
                }
            }
        }

        if (DataCenter::get_instance()->config.silent_mapping() &&
            DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
        {
            static std::double_t start_timestamp = 0.0;

            if (!feed_ && !input_measurements_.empty() && input_measurements_.back().rtk_msgs.front()->rtk_status == 0)
            {
                feed_           = true;
                start_timestamp = input_measurements_.back().rtk_msgs.front()->timestamp - 15.0;
            }

            if (!feed_ && !input_measurements_.empty())
            {
                LOG_INFO << "wait for: rtk_status == 0 , current rtk_status is "
                         << input_measurements_.back().rtk_msgs.front()->rtk_status;

                std::cout << "wait for: rtk_status == 0 , current rtk_status is "
                          << input_measurements_.back().rtk_msgs.front()->rtk_status << std::endl;
            }

            if (!input_measurements_.empty() && feed_)
            {
                while (!input_measurements_.empty() &&
                       input_measurements_.front().rtk_msgs.front()->timestamp < start_timestamp)
                {
                    input_measurements_.pop_front();
                }

                while (!input_measurements_.empty() && input_measurements_.size() > 200)
                {
                    input_measurements_.pop_front();
                }
            }
        }
        else
        {
            feed_ = true;
        }

        // step3 feed sync data into algorithm
        if (lio_estimator_ptr_ != nullptr && !input_measurements_.empty() && feed_)
        {
            auto measurement_input = input_measurements_.front();

            std::sort(measurement_input.msg_queue.begin(), measurement_input.msg_queue.end(), parking_slam::MsgCMP());

            bool ret_1 = true;
            bool ret_2 = true;
            for (const auto& msg : measurement_input.msg_queue)
            {
                if (msg->get_type() == parking_slam::MsgType::RTK)
                {
                    auto rtk_msg = std::dynamic_pointer_cast<parking_slam::RtkMsg>(msg);
                    ret_1        = lio_estimator_ptr_->process_imu_msg(rtk_msg);
                }
                else if (msg->get_type() == parking_slam::MsgType::WheelSpeed)
                {
                    auto wheel_speed_msg = std::dynamic_pointer_cast<parking_slam::WheelSpeedMsg>(msg);
                    ret_2                = lio_estimator_ptr_->process_wheel_msg(wheel_speed_msg);
                }
            }

            if (!ret_1 || !ret_2)
            {
                // continue;
                return 0;
            }

            parking_slam::Timer timer;
            timer.Tic();
            parking_slam::LIOState             state(0U);
            std::deque<parking_slam::LIOFrame> key_frame_deque;
            std::deque<parking_slam::LIOState> lio_state_deque;

            if (lio_estimator_ptr_->process_point_cloud(measurement_input.lidar_msg, state, key_frame_deque,
                                                        lio_state_deque))
            {
                timer.Toc();
                LOG_INFO << "fusion cost: " << timer.Duration();
                DataCenter::get_instance()->set_current_lio_state(state);
                DataCenter::get_instance()->set_current_lio_state_deque(lio_state_deque);
                DataCenter::get_instance()->set_current_key_frame_deque(key_frame_deque);

                if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
                {
                    DataCenter::get_instance()->boot_llh = lio_estimator_ptr_->get_boot_llh();
                }
            }

            input_measurements_.pop_front();
        }
        // else
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ParkingSLAMFusionExec::create_lio_estimator_ptr()
{
    if (lio_estimator_ptr_ != nullptr)
    {
        LOG_ERROR << "lio_estimator_ptr_ is not null!";

        return true;
    }

    if (!DataCenter::get_instance()->inited())
    {
        LOG_ERROR << "data center is not inited!";
        return false;
    }

    if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
    {
        LOG_INFO << "start MAPPING ";

        lio_estimator_ptr_ = std::make_shared<parking_slam::LIOEstimator>(DataCenter::get_instance()->config,
                                                                          DataCenter::get_instance()->hardware_config,
                                                                          parking_slam::Mode::MAPPING);
    }
    else if (DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::LOCALIZATION)
    {
        LOG_INFO << "start LOCALIZATION ";

        if (DataCenter::get_instance()->map_loaded())
        {
            lio_estimator_ptr_ = std::make_shared<parking_slam::LIOEstimator>(
                DataCenter::get_instance()->config, DataCenter::get_instance()->hardware_config,
                parking_slam::Mode::LOCALIZATION);
            lio_estimator_ptr_->set_map(DataCenter::get_instance()->map);
        }
        else
        {
            lio_estimator_ptr_.reset();
            LOG_ERROR << "map is not inited!";

            return false;
        }
    }
    else
    {
        return false;
    }

    inited_ = true;
    return true;
}

}  // namespace parking_slam
