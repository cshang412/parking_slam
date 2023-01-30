#include "data_center.h"

#include "common/type_converter.h"
#include "map_manager.h"

namespace parking_slam
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DataCenter::DataCenter() : point_cloud_msg_deque(50), wheel_speed_msg_deque(500), rtk_msg_deque(500), lio_state_(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DataCenter* DataCenter::get_instance()
{
    static DataCenter instance;
    return &instance;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::load_hardware_config()
{
    ::haomo::hios::HardwareConfig _hw_config;
    if (!haomo::pos::param_server::query<haomo::hios::HardwareConfig>(FLAGS_hardware_config, _hw_config))
    {
        LOG_ERROR << "get hardware config from pos failed";
        return false;
    }

    Json::Value root;
    {
        std::unordered_map<::haomo::hios::LidarParameter_LidarName, std::string> um;
        um[haomo::hios::LidarParameter_LidarName_FRONT_LEFT_LIDAR]  = "left_M1_lidar_data";
        um[haomo::hios::LidarParameter_LidarName_FRONT_RIGHT_LIDAR] = "right_M1_lidar_data";
        um[haomo::hios::LidarParameter_LidarName_MIDDLE_LIDAR]      = "center_128_lidar_data";
        for (auto& item : _hw_config.sensor_config().lidar_param())
        {
            if (!um.count(item.name()))
            {
                continue;
            }

            auto sensor_name                   = um[item.name()];
            root[sensor_name]["attitude"]["w"] = item.pose().attitude().w();
            root[sensor_name]["attitude"]["x"] = item.pose().attitude().x();
            root[sensor_name]["attitude"]["y"] = item.pose().attitude().y();
            root[sensor_name]["attitude"]["z"] = item.pose().attitude().z();

            root[sensor_name]["translation"]["x"] = item.pose().translation().x();
            root[sensor_name]["translation"]["y"] = item.pose().translation().y();
            root[sensor_name]["translation"]["z"] = item.pose().translation().z();
        }
    }

    {
        std::unordered_map<haomo::hios::CameraParameter_CameraNameType, std::string> um;
        um[haomo::hios::CameraParameter_CameraNameType_front_short_camera]   = "front_short_camera";
        um[haomo::hios::CameraParameter_CameraNameType_front_fisheye_camera] = "front_fisheye_camera";
        um[haomo::hios::CameraParameter_CameraNameType_left_fisheye_camera]  = "left_fisheye_camera";
        um[haomo::hios::CameraParameter_CameraNameType_right_fisheye_camera] = "right_fisheye_camera";
        um[haomo::hios::CameraParameter_CameraNameType_rear_fisheye_camera]  = "rear_fisheye_camera";
        for (auto& item : _hw_config.sensor_config().cam_param())
        {
            if (!um.count(item.name()))
            {
                continue;
            }

            auto sensor_name = um[item.name()];

            root[sensor_name]["attitude"]["w"] = item.pose().attitude().w();
            root[sensor_name]["attitude"]["x"] = item.pose().attitude().x();
            root[sensor_name]["attitude"]["y"] = item.pose().attitude().y();
            root[sensor_name]["attitude"]["z"] = item.pose().attitude().z();

            root[sensor_name]["translation"]["x"] = item.pose().translation().x();
            root[sensor_name]["translation"]["y"] = item.pose().translation().y();
            root[sensor_name]["translation"]["z"] = item.pose().translation().z();

            root[sensor_name]["height"] = item.image_height();
            root[sensor_name]["width"]  = item.image_width();

            root[sensor_name]["fx"] = item.fx();
            root[sensor_name]["fy"] = item.fy();
            root[sensor_name]["cx"] = item.cx();
            root[sensor_name]["cy"] = item.cy();

            for (int di = 0; di < item.distortion_size(); ++di)
            {
                root[sensor_name]["distortion"][di] = item.distortion(di);
            }
        }
    }

    {
        root["rtk"]["attitude"]["w"] = _hw_config.sensor_config().rtk_param().pose().attitude().w();
        root["rtk"]["attitude"]["x"] = _hw_config.sensor_config().rtk_param().pose().attitude().x();
        root["rtk"]["attitude"]["y"] = _hw_config.sensor_config().rtk_param().pose().attitude().y();
        root["rtk"]["attitude"]["z"] = _hw_config.sensor_config().rtk_param().pose().attitude().z();

        root["rtk"]["translation"]["x"] = _hw_config.sensor_config().rtk_param().pose().translation().x();
        root["rtk"]["translation"]["y"] = _hw_config.sensor_config().rtk_param().pose().translation().y();
        root["rtk"]["translation"]["z"] = _hw_config.sensor_config().rtk_param().pose().translation().z();
    }

    hardware_config.load_from_json(root);

    LOG_WARNING << "load hardware config success!";

    hardware_config_init_ = true;

    return hardware_config_init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::load_config(const std::string& config_path)
{
    if (config_init_)
    {
        return true;
    }
    if (!::haomo::pos::read_proto_from_text_file(config_path.c_str(), &config))
    {
        LOG_ERROR << "load config file error!";
        return false;
    }
    config.PrintDebugString();
    config_init_ = true;
    return config_init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::config_inited() const
{
    return config_init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::load_map()
{
    if (!config_init_)
    {
        config_init_ = false;

        LOG_ERROR << "wait for config loaded!";

        return false;
    }

    map_init_ = MapManager::load_map(config.map_process_config(), avp_map, map, color_map);
    if (map.init)
    {
        map_init_ = true;

        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::inited() const
{
    return hardware_config_init_ & config_init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::lio_inited() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return lio_init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataCenter::map_loaded() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return map_init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
parking_slam::LIOState DataCenter::get_current_lio_state() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return lio_state_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataCenter::set_current_lio_state(const parking_slam::LIOState& lio_state)
{
    std::lock_guard<std::mutex> lock(mutex_);

    lio_state_ = lio_state;

    lio_init_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::Vector3d> DataCenter::get_ref_line() const
{
    std::lock_guard<std::mutex> lock(ref_line_mutex_);

    return current_ref_line_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataCenter::set_ref_line(const std::vector<Eigen::Vector3d>& ref_line)
{
    std::lock_guard<std::mutex> lock(ref_line_mutex_);

    current_ref_line_ = ref_line;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const std::deque<parking_slam::LIOFrame>& DataCenter::get_current_key_frame_deque() const
{
    std::lock_guard<std::mutex> lock(key_frames_mutex_);

    return key_frame_deque_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataCenter::set_current_key_frame_deque(const std::deque<parking_slam::LIOFrame>& key_frame_deque)
{
    std::lock_guard<std::mutex> lock(key_frames_mutex_);

    key_frame_deque_ = key_frame_deque;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const std::deque<parking_slam::LIOState>& DataCenter::get_current_lio_state_deque() const
{
    std::lock_guard<std::mutex> lock(lio_states_mutex_);

    return LIO_state_deque_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataCenter::set_current_lio_state_deque(const std::deque<parking_slam::LIOState>& lio_state_deque)
{
    std::lock_guard<std::mutex> lock(lio_states_mutex_);

    LIO_state_deque_ = lio_state_deque;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const std::deque<parking_slam::LIOState>& DataCenter::get_predict_state_deque() const
{
    std::lock_guard<std::mutex> lock(predict_states_mutex_);

    return predict_state_deque_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataCenter::set_predict_state(const parking_slam::LIOState& predict_state)
{
    std::lock_guard<std::mutex> lock(predict_states_mutex_);

    if (predict_state_deque_.size() > 2)
    {
        auto riter      = predict_state_deque_.rbegin();
        auto rnext_iter = std::next(riter);

        Eigen::Isometry3d T_world_body_i = parking_slam::TypeConverter::nav_state_to_iso3d(riter->get_nav_state());
        Eigen::Isometry3d T_world_body_j =
        parking_slam::TypeConverter::nav_state_to_iso3d(rnext_iter->get_nav_state());

        double distance = (T_world_body_i.translation() - T_world_body_j.translation()).norm();

        if (distance <= 0.3)
        {
            predict_state_deque_.pop_back();
        }

        predict_state_deque_.push_back(predict_state);
    }
    else
    {
        predict_state_deque_.push_back(predict_state);
    }
}

}  // namespace parking_slam
