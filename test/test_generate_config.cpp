#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <fstream>
#include <iostream>

#include "config.pb.h"
#include "data.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    std::string                     file_path = "/home/ubuntu/haomo/data/config.prototxt";
    parking_slam::ParkingSLAMConfig config;
    config.set_mode(parking_slam::Mode::MAPPING);

    config.mutable_sensor_config()->set_lidar_type(parking_slam::LidarType::M1);
    config.mutable_sensor_config()->set_acc_noise(1e-1);
    config.mutable_sensor_config()->set_gyr_noise(1e-2);
    config.mutable_sensor_config()->set_acc_w_noise(5e-3);
    config.mutable_sensor_config()->set_gyr_w_noise(1e-4);

    config.mutable_registration_config()->set_registration_type(parking_slam::RegistrationType::VOXEL_GICP);
    config.mutable_registration_config()->set_thresh_icp_max_iteration(35);
    config.mutable_registration_config()->set_thresh_icp_error(0.001);
    config.mutable_registration_config()->set_thresh_fitness_score(0.5);
    config.mutable_registration_config()->set_thresh_overlap(0.5);
    config.mutable_registration_config()->set_thresh_invalid_registration_ratio(0.5);
    config.mutable_registration_config()->set_thresh_init_loc_fitness_score(0.5);
    config.mutable_registration_config()->set_thresh_init_loc_overlap(0.5);
    config.mutable_registration_config()->set_thresh_diff_angle(1.0);
    config.mutable_registration_config()->set_thresh_diff_translation(1.0);
    config.mutable_registration_config()->set_thresh_diff_angle_graph_build(1.0);
    config.mutable_registration_config()->set_thresh_diff_translation_graph_build(1.0);

    config.mutable_graph_builder_config()->set_thresh_keyframe_distance(20.0);
    config.mutable_graph_builder_config()->set_thresh_keyframe_yaw(360.0);
    config.mutable_graph_builder_config()->set_thresh_odometry_distance(0.1);
    config.mutable_graph_builder_config()->set_thresh_odometry_angle(1.0);
    config.mutable_graph_builder_config()->set_thresh_loop_distance(0.3);
    config.mutable_graph_builder_config()->set_thresh_loop_angle(3.0);
    config.mutable_graph_builder_config()->set_thresh_loc_distance(1.0);
    config.mutable_graph_builder_config()->set_thresh_loc_yaw(1.0);

    config.mutable_graph_builder_config()->set_enable_marginalized(true);
    config.mutable_graph_builder_config()->set_enable_loop_closure_factor(true);
    config.mutable_graph_builder_config()->set_enable_pose_between_factor(true);
    config.mutable_graph_builder_config()->set_enable_keyframe_to_map_factor(true);

    config.mutable_graph_builder_config()->set_enable_imu_factor(true);
    config.mutable_graph_builder_config()->set_enable_wheel_scale_factor(true);
    config.mutable_graph_builder_config()->set_enable_imu_bias_factor(true);
    config.mutable_graph_builder_config()->set_enable_zero_velocity_factor(true);
    config.mutable_graph_builder_config()->set_enable_zero_rot_bias_velocity_factor(true);
    config.mutable_graph_builder_config()->set_enable_body_velocity_factor(true);

    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_pose_between_noise()->set_r(1e-6);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_pose_between_noise()->set_t(5e-3);

    config.mutable_graph_builder_config()->mutable_factor_noise()->set_zero_velocity_noise(1e-4);

    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_zero_rot_bias_velocity_noise()->set_acc(
        1e-2);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_zero_rot_bias_velocity_noise()->set_gyr(
        1e-6);

    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_body_velocity_noise()->set_x(4e-2);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_body_velocity_noise()->set_y(1e-2);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_body_velocity_noise()->set_z(1e-4);

    config.mutable_graph_builder_config()->mutable_factor_noise()->set_wheel_scale_noise(1e-4);

    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_prior_pose_noise()->set_r_x(3e-4);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_prior_pose_noise()->set_r_y(3e-4);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_prior_pose_noise()->set_r_z(3e-4);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_prior_pose_noise()->set_t_x(1e-4);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_prior_pose_noise()->set_t_y(1e-4);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_prior_pose_noise()->set_t_z(1e-4);

    config.mutable_graph_builder_config()->mutable_factor_noise()->set_prior_imu_bias_noise(1e-3);
    config.mutable_graph_builder_config()->mutable_factor_noise()->set_prior_velocity_noise(4e-2);
    config.mutable_graph_builder_config()->mutable_factor_noise()->set_prior_wheel_scale_noise(1e-3);

    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_loop_noise()->set_r(1e-6);
    config.mutable_graph_builder_config()->mutable_factor_noise()->mutable_loop_noise()->set_t(5e-3);

    config.mutable_graph_builder_config()->set_max_num_state_in_graph(-1);

    config.mutable_loop_closure_detection_config()->set_thresh_timestamp_diff(20.0);
    config.mutable_loop_closure_detection_config()->set_thresh_distance(40.0);
    config.mutable_loop_closure_detection_config()->set_thresh_z_diff(2.0);
    config.mutable_loop_closure_detection_config()->set_thresh_total_length(11.0);
    config.mutable_loop_closure_detection_config()->set_thresh_iou(0.4);

    config.mutable_initialize_config()->set_thresh_global_pitch_angle(0.9);
    config.mutable_initialize_config()->set_nearest_ksearch_num(3);
    config.mutable_initialize_config()->set_thresh_nearest_ksearch_max_angle(1.0);
    config.mutable_initialize_config()->set_thresh_nearest_ksearch_max_dist(1.0);

    config.set_visualizer_show_map(true);

    config.mutable_map_process_config()->set_map_path("/home/ubuntu/haomo/data/result");
    config.mutable_map_process_config()->set_max_distance(60.0);
    config.mutable_map_process_config()->set_max_z(2.0);
    config.mutable_map_process_config()->set_map_type(parking_slam::MapType::PROTO32);
    config.mutable_map_process_config()->set_save_cloud_ply(true);
    config.mutable_map_process_config()->set_save_trajectory_ply(true);
    config.mutable_map_process_config()->set_save_avp_map(true);
    config.mutable_map_process_config()->set_saved_max_map_num(10);
    config.mutable_map_process_config()->set_saved_map_database_name("/map");
    config.mutable_map_process_config()->set_saved_map_name("2022-12-09-001");
    config.set_silent_mapping(true);

    config.mutable_rpc_config()->set_enable(true);
    config.mutable_rpc_config()->set_rpc_ip("127.0.0.1");

    std::string str_config;
    google::protobuf::TextFormat::PrintToString(config, &str_config);
    std::ofstream ofs;
    ofs.open(file_path, std::ios::out | std::ios_base::ate);

    if (!ofs.is_open())
    {
        std::cerr << "open config file error!" << file_path << std::endl;
        return -1;
    }
    ofs << str_config;
    ofs.flush();
    ofs.close();

    parking_slam::ParkingSLAMConfig config2;
    if (!::haomo::pos::read_proto_from_text_file(file_path.c_str(), &config2))
    {
        std::cerr << "load config file error!" << std::endl;
    }

    config2.PrintDebugString();

    return 0;
}
