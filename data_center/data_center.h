#pragma once

#include "common/msg/point_cloud_msg.h"
#include "common/msg/rtk_msg.h"
#include "common/msg/wheel_speed_msg.h"
#include "concurrent_deque.h"
#include "data.h"
#include "data_provider/hardware_config.h"
#include "lio_frame.h"
#include "lio_state.h"
#include "map_manager.h"

namespace parking_slam
{
/**
 * @brief
 *
 */
class StateMachine
{
public:
    //@todo chenghe
    //加入建图与定位状态切换
    // 1. Mapping ==> Localization: STOP => RESET => LOAD Map =>START
    // 2. Localization ==> Mapping: STOP => RESET => START
    enum class Status
    {
        START,
        STOP,
        RESET,
    };

    /**
     * @brief Construct a new State Machine object
     *
     */
    StateMachine() : mode_(parking_slam::Mode::MAPPING), current_status_(Status::STOP)
    {
    }

    /**
     * @brief Get the mode object
     *
     * @return parking_slam::Mode
     */
    parking_slam::Mode get_mode() const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        return mode_;
    }

    /**
     * @brief Get the status object
     *
     * @return Status
     */
    Status get_status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        return current_status_;
    }

    /**
     * @brief Set the mode object
     *
     * @param mode
     */
    void set_mode(const parking_slam::Mode& mode)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        mode_ = mode;
    }

    /**
     * @brief Set the status object
     *
     * @param status
     */
    void set_status(const Status& status)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        current_status_ = status;
    }

private:
    /**
     * @brief
     *
     */
    parking_slam::Mode mode_;

    Status current_status_;

    mutable std::mutex mutex_;
};

/**
 * @brief
 *
 */
class DataCenter
{
public:
    /**
     * @brief Construct a new Data Center object
     *
     */
    explicit DataCenter();

    /**
     * @brief Get the instance object
     *
     * @return DataCenter*
     */
    static DataCenter* get_instance();

    /**
     * @brief Destroy the Data Center object
     *
     */
    ~DataCenter() = default;

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool load_hardware_config();

    /**
     * @brief
     *
     * @param config_path
     * @return true
     * @return false
     */
    bool load_config(const std::string& config_path);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool load_map();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool inited() const;

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool lio_inited() const;

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool map_loaded() const;

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool config_inited() const;

    /**
     * @brief Get the current lio state object
     *
     * @return parking_slam::LIOState
     */
    parking_slam::LIOState get_current_lio_state() const;

    /**
     * @brief Set the current lio state object
     *
     * @param lio_state
     */
    void set_current_lio_state(const parking_slam::LIOState& lio_state);

    /**
     * @brief Get the current key frame deque object
     *
     * @return const std::deque<parking_slam::LIOFrame>&
     */
    const std::deque<parking_slam::LIOFrame>& get_current_key_frame_deque() const;

    /**
     * @brief Set the current key frame deque object
     *
     * @param key_frame_deque
     */
    void set_current_key_frame_deque(const std::deque<parking_slam::LIOFrame>& key_frame_deque);

    /**
     * @brief Get the current lio state deque object
     *
     * @return const std::deque<parking_slam::LIOState>&
     */
    const std::deque<parking_slam::LIOState>& get_current_lio_state_deque() const;

    /**
     * @brief Set the current lio state deque object
     *
     * @param lio_state_deque
     */
    void set_current_lio_state_deque(const std::deque<parking_slam::LIOState>& lio_state_deque);

    /**
     * @brief Get the predict state deque object
     *
     * @return const std::deque<parking_slam::LIOState>&
     */
    const std::deque<parking_slam::LIOState>& get_predict_state_deque() const;

    /**
     * @brief Set the predict state object
     *
     * @param predict_state
     */
    void set_predict_state(const parking_slam::LIOState& predict_state);

    void set_ref_line(const std::vector<Eigen::Vector3d>& ref_line);

    std::vector<Eigen::Vector3d> get_ref_line() const;

    /**
     * @brief
     *
     * @return const StateMachine&
     */
    const StateMachine& state_machine() const
    {
        return state_machine_;
    }

    /**
     * @brief
     *
     * @return StateMachine&
     */
    StateMachine& state_machine()
    {
        return state_machine_;
    }

    /**
     * @brief
     *
     */
    void reset_lio_data()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lio_init_ = false;
        {
            std::lock_guard<std::mutex> lg(key_frames_mutex_);
            key_frame_deque_.clear();
        }

        {
            std::lock_guard<std::mutex> lg(lio_states_mutex_);
            LIO_state_deque_.clear();
        }

        {
            std::lock_guard<std::mutex> lg(predict_states_mutex_);
            predict_state_deque_.clear();
        }
    }

    /**
     * @brief
     *
     */
    void clear_sensor_data()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        point_cloud_msg_deque.clear();
        wheel_speed_msg_deque.clear();
        rtk_msg_deque.clear();
    }

    /**
     * @brief
     *
     */
    ConcurrentDeque<parking_slam::MultiSyncedPointCloudMsg::Ptr> point_cloud_msg_deque;
    ConcurrentDeque<parking_slam::WheelSpeedMsg::Ptr>            wheel_speed_msg_deque;
    ConcurrentDeque<parking_slam::RtkMsg::Ptr>                   rtk_msg_deque;
    parking_slam::HardwareConfig                                 hardware_config;
    parking_slam::ParkingSLAMConfig                              config;
    parking_slam::LIOMap                                         map;
    pcl::PointCloud<pcl::PointXYZRGB>                            color_map;
    haomo::avp_map::AVPMapMsg                                    avp_map;
    Eigen::Vector3d                                              boot_llh;
    bool                                                         save_map = false;
    bool                                                         start    = false;

private:
    /**
     * @brief
     *
     */
    bool hardware_config_init_ = false;

    bool config_init_ = false;

    bool lio_init_ = false;

    bool map_init_ = false;

    parking_slam::LIOState             lio_state_;
    std::deque<parking_slam::LIOFrame> key_frame_deque_;
    std::deque<parking_slam::LIOState> LIO_state_deque_;
    std::deque<parking_slam::LIOState> predict_state_deque_;
    StateMachine                       state_machine_;
    std::vector<Eigen::Vector3d>       current_ref_line_;

    mutable std::mutex mutex_;
    mutable std::mutex key_frames_mutex_;
    mutable std::mutex lio_states_mutex_;
    mutable std::mutex predict_states_mutex_;
    mutable std::mutex ref_line_mutex_;
};
}  // namespace parking_slam
