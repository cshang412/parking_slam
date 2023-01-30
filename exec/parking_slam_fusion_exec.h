#pragma once
#include <iostream>
#include <typeinfo>

#include "data.h"
#include "data_center/data_center.h"
#include "lio_localization.h"
#include "lio_mapping.h"

namespace parking_slam
{

class ParkingSLAMFusionExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief
     *
     */
    struct Measurement
    {
        std::vector<parking_slam::RtkMsg::Ptr>        rtk_msgs;
        std::vector<parking_slam::BaseMsg::Ptr>       msg_queue;
        std::vector<parking_slam::WheelSpeedMsg::Ptr> wheel_speed_msgs;
        parking_slam::MultiSyncedPointCloudMsg::Ptr   lidar_msg;
    };

    /**
     * @brief Construct a new Parking S L A M Fusion Exec object
     *
     * @param name
     * @param type
     */
    ParkingSLAMFusionExec(const std::string& name, const std::string& type) : IExec(name, type)
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
        config_file_name_ = config_dir + "/parking_slam/config/config.prototxt";
        feed_ = false;

        DataCenter::get_instance()->load_config(config_file_name_);

        // stop_   = false;
        // thread_ = std::thread(std::bind(&ParkingSLAMFusionExec::run, this));
        // thread_.detach();

        return 0;
    }

    /**
     * @brief
     *
     * @return int
     */
    int release()
    {
        // stop_ = true;
        // if (thread_.joinable())
        // {
        //     thread_.join();
        // }

        return 0;
    }

private:
    /**
     * @brief
     *
     */
    void reset_data();

    /**
     * @brief Create a lio estimator ptr object
     *
     * @return true
     * @return false
     */
    bool create_lio_estimator_ptr();

    // /**
    //  * @brief
    //  *
    //  */
    // void run();

private:
    /**
     * @brief
     *
     */
    // std::atomic<bool>               stop_;
    // std::thread                     thread_;
    bool                            inited_ = false;
    parking_slam::LIOEstimator::Ptr lio_estimator_ptr_;
    std::deque<Measurement>         input_measurements_;
    bool                            feed_;

    std::string config_file_name_;
};
}  // namespace parking_slam
