#pragma once

#include <iostream>
#include <typeinfo>

#include "data.h"
#include "data_center/data_center.h"

namespace parking_slam
{

/**
 * @brief
 *
 */
class RTKWheelSpeedExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new RTKWheelSpeedExec object
     *
     * @param name
     * @param type
     */
    RTKWheelSpeedExec(const std::string& name, const std::string& type) : IExec(name, type), current_predict_state_(0)
    {
    }

    /**
     * @brief
     *
     * @param input
     * @param output
     * @return int
     */
    int process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output);

    /**
     * @brief
     *
     * @param config_dir
     * @return int
     */
    int init(const std::string& config_dir)
    {
        haomo::pos::Module::get_instance()->regist("PSLAM");

        return 0;
    }

    /**
     * @brief
     *
     * @return int
     */
    int release()
    {
        LOG_WARNING << "In TestPeriodicalExec release";
        return 0;
    }

private:
    void predict(const parking_slam::RtkMsg::Ptr& imu_msg, const parking_slam::WheelSpeedMsg::Ptr& wheel_msg);

    bool                                         current_predict_state_valid_ = false;
    parking_slam::LIOState                       current_predict_state_;
    std::deque<parking_slam::RtkMsg::Ptr>        rtk_msgs_;
    std::deque<parking_slam::WheelSpeedMsg::Ptr> wheel_speed_msgs_;
};

/**
 * @brief
 *
 */
class RTKExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new RTKWheelSpeedExec object
     *
     * @param name
     * @param type
     */
    RTKExec(const std::string& name, const std::string& type) : IExec(name, type)
    {
    }

    /**
     * @brief
     *
     * @param input
     * @param output
     * @return int
     */
    int process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output);

    /**
     * @brief
     *
     * @param config_dir
     * @return int
     */
    int init(const std::string& config_dir)
    {
        haomo::pos::Module::get_instance()->regist("PSLAM");

        return 0;
    }

    /**
     * @brief
     *
     * @return int
     */
    int release()
    {
        LOG_WARNING << "In TestPeriodicalExec release";
        return 0;
    }
};

/**
 * @brief
 *
 */
class WheelSpeedExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new WheelSpeedExec object
     *
     * @param name
     * @param type
     */
    WheelSpeedExec(const std::string& name, const std::string& type) : IExec(name, type)
    {
    }

    /**
     * @brief
     *
     * @param input
     * @param output
     * @return int
     */
    int process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output);

    /**
     * @brief
     *
     * @param config_dir
     * @return int
     */
    int init(const std::string& config_dir)
    {
        haomo::pos::Module::get_instance()->regist("PSLAM");

        return 0;
    }

    /**
     * @brief
     *
     * @return int
     */
    int release()
    {
        LOG_WARNING << "In TestPeriodicalExec release";
        return 0;
    }
};
}  // namespace parking_slam
