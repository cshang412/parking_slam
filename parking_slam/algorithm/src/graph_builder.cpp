/**
 * @file graph_builder.cpp
 * @brief
 * @author zhaoqiang
 * @author chenghe
 * @date 2022-08-11
 */

#include "graph_builder.h"

#include "common/timer.h"
#include "math/frame_distance_and_yaw_computer.h"

namespace parking_slam
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GraphBuilder::GraphBuilder(const ParkingSLAMConfig& config, const HardwareConfig& hardware_config, const Mode& mode)
  : mode_(mode)
{
    config_.CopyFrom(config);
    hardware_config_       = hardware_config;
    lidar_odometry_        = std::make_shared<LidarOdometry>(config_, hardware_config_);
    loop_closure_detector_ = std::make_shared<LoopClosureDetector>(config_, hardware_config_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GraphBuilder::Problem GraphBuilder::build_graph(const LIOFrame& frame, const std::deque<LIOFrame>& key_frame_deque,
                                                const std::deque<RtkMsg::Ptr>&             imu_deque,
                                                const std::deque<WheelSpeedMsg::Ptr>&      wheel_speed_deque,
                                                const gtsam::PreintegratedImuMeasurements& pim, double sum_wheel_speed,
                                                bool sum_speed_valid) const
{
    Problem problem;

    // insert values
    problem.graph_values.insert(frame.state.get_pose_key(), frame.state.get_nav_state().pose());
    problem.graph_values.insert(frame.state.get_velocity_key(), frame.state.get_nav_state().velocity());
    problem.graph_values.insert(frame.state.get_bias_key(), frame.state.get_imu_bias());
    problem.graph_values.insert(frame.state.get_wheel_scale_key(), frame.state.get_wheel_scale());

    if (key_frame_deque.empty())
    {
        create_prior_pose_factor(frame, problem.factor_graph);
        create_prior_imu_bias_factor(frame, problem.factor_graph);
        create_prior_velocity_factor(frame, problem.factor_graph);
        create_prior_wheel_scale_factor(frame, problem.factor_graph);

        return problem;
    }

    // add imu factor
    if (config_.graph_builder_config().enable_imu_factor())
    {
        create_imu_factor(key_frame_deque.back(), frame, pim, problem.factor_graph);
    }

    // add imu bias factor
    if (config_.graph_builder_config().enable_imu_bias_factor())
    {
        create_imu_bias_between_factor(key_frame_deque.back(), frame, pim.deltaTij(), problem.factor_graph);
    }

    // add zero speed pose between factor
    if (config_.graph_builder_config().enable_pose_between_factor() && sum_speed_valid && sum_wheel_speed <= 1e-8 &&
        !key_frame_deque.empty())
    {
        create_pose_between_factor(key_frame_deque.back(), frame,
                                   config_.graph_builder_config().factor_noise().pose_between_noise().r(),
                                   config_.graph_builder_config().factor_noise().pose_between_noise().t(),
                                   gtsam::Pose3::identity(), problem.factor_graph);
    }

    auto wheel_speed = wheel_speed_deque.back()->vehicle_speed;

    // add velocity factor
    if (config_.graph_builder_config().enable_zero_velocity_factor() && std::abs(wheel_speed) < 1e-3)
    {
        create_zero_velocity_factor(frame, problem.factor_graph);
    }
    else if (config_.graph_builder_config().enable_zero_rot_bias_velocity_factor() && std::abs(wheel_speed) < 1e-3)
    {
        create_zero_rot_bias_velocity_factor(frame, imu_deque.back(), problem.factor_graph);
    }
    else if (config_.graph_builder_config().enable_body_velocity_factor() && std::abs(wheel_speed) >= 1e-3)
    {
        create_body_velocity_factor(frame, wheel_speed_deque.back(), hardware_config_.get_body_T_rtk(),
                                    problem.factor_graph);
    }

    // add wheel scale factor
    if (config_.graph_builder_config().enable_wheel_scale_factor() && !key_frame_deque.empty())
    {
        create_wheel_scale_factor(key_frame_deque.back(), frame, problem.factor_graph);
    }

    static LIOState last_state = frame.state;

    // add odometry factor
    if (!frame.is_car_static && mode_ != Mode::LOCALIZATION)
    {
        double distance, yaw, z;
        compute_frame_distance_and_yaw(last_state, frame.state, hardware_config_.get_rtk_T_body(), distance, yaw, z);

        if (!(distance < config_.graph_builder_config().thresh_odometry_distance() &&
              yaw < config_.graph_builder_config().thresh_odometry_angle()))
        {
            last_state = frame.state;

            if (config_.graph_builder_config().enable_pose_between_factor() && key_frame_deque.size() > 2)
            {
                Timer timer;
                timer.Tic();

                auto riter      = key_frame_deque.rbegin();
                auto rnext_iter = std::next(riter);
                create_odom_constrain_factor(
                    frame, *rnext_iter, config_.graph_builder_config().factor_noise().pose_between_noise().r(),
                    config_.graph_builder_config().factor_noise().pose_between_noise().t(),
                    problem.invalid_registration_ratio, problem.pose_between, problem.factor_graph);

                timer.Toc();
                LOG_INFO << "add odom factor cost: " << timer.Duration() << " second.";
            }
        }
    }

    static LIOState last_loop_state = frame.state;

    // add loop closure factor
    if (config_.graph_builder_config().enable_loop_closure_factor() && !frame.is_car_static &&
        mode_ != Mode::LOCALIZATION)
    {
        double distance, yaw, z;
        compute_frame_distance_and_yaw(last_loop_state, frame.state, hardware_config_.get_rtk_T_body(), distance, yaw,
                                       z);
        if (!(distance < config_.graph_builder_config().thresh_loop_distance() &&
              yaw < config_.graph_builder_config().thresh_loop_angle()))
        {
            last_loop_state = frame.state;
            Timer timer;
            timer.Tic();

            auto loop_index = loop_closure_detector_->detect_loop_closure(frame, key_frame_deque);

            if (loop_index > 0 && loop_index < static_cast<std::int32_t>(key_frame_deque.size()))
            {
                gtsam::Pose3 pose_between;

                create_odom_constrain_factor(frame, key_frame_deque.at(loop_index),
                                             config_.graph_builder_config().factor_noise().pose_between_noise().r(),
                                             config_.graph_builder_config().factor_noise().pose_between_noise().t(),
                                             problem.invalid_registration_ratio, pose_between, problem.factor_graph);

                problem.detected_loop_closure = true;

                timer.Toc();
                LOG_INFO << "add loop factor cost: " << timer.Duration() << " second.";
            }
        }
    }

    auto   riter      = key_frame_deque.rbegin();
    auto   rnext_iter = std::next(riter);
    double distance, yaw, z;

    if (rnext_iter != key_frame_deque.rend())
    {
        compute_frame_distance_and_yaw(riter->state, rnext_iter->state, hardware_config_.get_rtk_T_body(), distance,
                                       yaw, z);
    }

    // add keyframe to map factor
    if (config_.graph_builder_config().enable_keyframe_to_map_factor() && key_frame_deque.size() > 2 &&
        !(distance < config_.graph_builder_config().thresh_keyframe_distance() &&
          yaw < config_.graph_builder_config().thresh_keyframe_yaw()) &&
        mode_ != Mode::LOCALIZATION)
    {
        auto rnext_next_iter = std::next(rnext_iter);

        if (rnext_next_iter != key_frame_deque.rend())
        {
            Timer timer;
            {
                timer.Tic();
                gtsam::Pose3 pose_between;
                create_odom_constrain_factor(*rnext_iter, *rnext_next_iter,
                                             config_.graph_builder_config().factor_noise().loop_noise().r(),
                                             config_.graph_builder_config().factor_noise().loop_noise().t(),
                                             problem.invalid_registration_ratio, pose_between, problem.factor_graph);
                timer.Toc();
                LOG_INFO << "add odom factor cost: " << timer.Duration() << " second.";

                problem.detected_loop_closure = true;
            }

            {
                timer.Tic();

                // add loop closure factor
                if (!frame.is_car_static && config_.graph_builder_config().enable_loop_closure_factor())
                {
                    auto loop_index = loop_closure_detector_->detect_loop_closure(*rnext_iter, key_frame_deque);

                    if (loop_index > 0 && loop_index < static_cast<std::int32_t>(key_frame_deque.size()))
                    {
                        gtsam::Pose3 pose_between;
                        create_odom_constrain_factor(*rnext_iter, key_frame_deque.at(loop_index),
                                                     config_.graph_builder_config().factor_noise().loop_noise().r(),
                                                     config_.graph_builder_config().factor_noise().loop_noise().t(),
                                                     problem.invalid_registration_ratio, pose_between,
                                                     problem.factor_graph);

                        problem.detected_loop_closure = true;

                        timer.Toc();
                        LOG_INFO << "[GraphBuilder::add_keyframe_to_map_factor] add loop factor cost: "
                                 << timer.Duration() << " second.";
                    }
                }
            }
        }
    }

    if (mode_ == Mode::LOCALIZATION)
    {
        static LIOState last_loc_state = frame.state;
        double          loc_distance, loc_yaw, loc_z;
        compute_frame_distance_and_yaw(last_loc_state, frame.state, hardware_config_.get_rtk_T_body(), loc_distance,
                                       loc_yaw, loc_z);
        if (loc_distance > config_.graph_builder_config().thresh_loc_distance() ||
            loc_yaw > config_.graph_builder_config().thresh_loc_yaw())
        {
            last_loc_state = frame.state;
            Timer timer;
            timer.Tic();
            create_absolute_prior_factor(frame, problem.invalid_registration_ratio, problem.factor_graph);
            timer.Toc();
            LOG_INFO << "[GraphBuilder::add_absolute_prior_factor] add absolute_prior_factor cost: " << timer.Duration()
                     << " second.";
        }
    }

    return problem;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool GraphBuilder::create_absolute_prior_factor(const LIOFrame& frame, double& invalid_registration_ratio,
                                                gtsam::NonlinearFactorGraph& factor_graph) const
{
    if (!map_.init)
    {
        return false;
    }

    Eigen::Isometry3d T_world_imu(frame.state.get_nav_state().pose().matrix());
    Eigen::Isometry3d T_world_body = T_world_imu * hardware_config_.get_rtk_T_body();
    VoxelRegistration voxel_matcher;
    voxel_matcher.set_force_update_corr_every_iter(true);
    voxel_matcher.set_source_voxel_map(frame.gaussian_voxel_map_ptr);
    voxel_matcher.set_target_voxel_map(map_.map.multi_gaussian_voxel_map_ptr);
    voxel_matcher.set_max_iterations(35);
    voxel_matcher.set_transformation_epsilon(0.001);
    Eigen::Matrix4f init_guess_matrix = T_world_body.cast<float>().matrix();
    voxel_matcher.compute_transformation(init_guess_matrix);
    Eigen::Isometry3d T_world_body_hat;
    T_world_body_hat = voxel_matcher.get_final_transformation().cast<double>();

    double fitness_score, overlap;
    voxel_matcher.get_fitness_score(fitness_score, overlap);

    GTPose3   pose_between(T_world_body_hat.matrix());
    GTPose3   init_pose  = GTPose3(T_world_body.matrix());
    GTPose3   diff       = init_pose * pose_between.inverse();
    GTVector3 rpy        = diff.rotation().rpy() * static_cast<double>(180.0) / static_cast<double>(M_PI);  // degree
    double    diff_angle = rpy.norm();
    double    diff_translation = diff.translation().vector().norm();

    static int all_cnt     = 0;
    static int invalid_cnt = 0;


    //  std::cout << "diff_angle: " << diff_angle << " ,diff_translation: " << diff_translation << std::endl;

    if (fitness_score < config_.registration_config().thresh_fitness_score() &&
        overlap < config_.registration_config().thresh_overlap() &&
        diff_angle > config_.registration_config().thresh_diff_angle_graph_build() &&
        diff_translation > config_.registration_config().thresh_diff_translation_graph_build())
    {
        invalid_cnt++;
        LOG_ERROR << "fitness_score or overlap NOT satisfy threshold, fitness_score: " << fitness_score
                  << " ,overlap: " << overlap << " ,diff_angle:" << diff_angle
                  << " ,diff_translation: " << diff_translation
                  << " threshold [ fitness_score: " << config_.registration_config().thresh_fitness_score()
                  << " ,overlap: " << config_.registration_config().thresh_overlap()
                  << " ,diff_angle: " << config_.registration_config().thresh_diff_angle_graph_build()
                  << " ,diff_translation: " << config_.registration_config().thresh_diff_translation_graph_build()
                  << " ]";
        return false;
    }

    Eigen::Isometry3d T_world_imu_hat = T_world_body_hat * hardware_config_.get_body_T_rtk();
    gtsam::Pose3      pose_prior(T_world_imu_hat.matrix());

    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 5e-3, 5e-3, 5e-3).finished());  // rad*rad, meter*meter (128_lidar 1e-4
                                                                               // 1e-4 1e-4)

    auto prior_pose_factor = gtsam::PriorFactor<gtsam::Pose3>(frame.state.get_pose_key(), pose_prior, prior_pose_noise);

    factor_graph.add(prior_pose_factor);

    all_cnt++;
    invalid_registration_ratio = static_cast<double>(invalid_cnt) / static_cast<double>(all_cnt);

    // std::cout << "fitness_score: " << fitness_score
    //           << " ,overlap: " << overlap << " ,invalid_registration_ratio: " << invalid_registration_ratio
    //           << std::endl;
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_imu_factor(const LIOFrame& old_frame, const LIOFrame& new_frame,
                                     const gtsam::PreintegratedImuMeasurements& pim,
                                     gtsam::NonlinearFactorGraph&               factor_graph) const
{
    auto imu_factor = gtsam::ImuFactor(old_frame.state.get_pose_key(), old_frame.state.get_velocity_key(),
                                       new_frame.state.get_pose_key(), new_frame.state.get_velocity_key(),
                                       old_frame.state.get_bias_key(), pim);
    factor_graph.add(imu_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_imu_bias_between_factor(const LIOFrame& old_frame, const LIOFrame& new_frame,
                                                  const double                 deltaTij,
                                                  gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        sqrt(deltaTij) * (gtsam::Vector(6) << config_.sensor_config().acc_w_noise(),
                          config_.sensor_config().acc_w_noise(), config_.sensor_config().acc_w_noise(),
                          config_.sensor_config().gyr_w_noise(), config_.sensor_config().gyr_w_noise(),
                          config_.sensor_config().gyr_w_noise())
                             .finished());

    auto imu_bias_between_factor = gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        old_frame.state.get_bias_key(), new_frame.state.get_bias_key(), gtsam::imuBias::ConstantBias(), noise);
    factor_graph.add(imu_bias_between_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_zero_velocity_factor(const LIOFrame& frame, gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto zupt_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(3) << config_.graph_builder_config().factor_noise().zero_velocity_noise(),
         config_.graph_builder_config().factor_noise().zero_velocity_noise(),
         config_.graph_builder_config().factor_noise().zero_velocity_noise())
            .finished());

    auto zero_velocity_factor = ZeroVelocityFactor(frame.state.get_velocity_key(), zupt_noise);
    factor_graph.add(zero_velocity_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_zero_rot_bias_velocity_factor(const LIOFrame& frame, const RtkMsg::Ptr& imu,
                                                        gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto bias_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << config_.graph_builder_config().factor_noise().zero_rot_bias_velocity_noise().acc(),
         config_.graph_builder_config().factor_noise().zero_rot_bias_velocity_noise().acc(),
         config_.graph_builder_config().factor_noise().zero_rot_bias_velocity_noise().acc(),
         config_.graph_builder_config().factor_noise().zero_rot_bias_velocity_noise().gyr(),
         config_.graph_builder_config().factor_noise().zero_rot_bias_velocity_noise().gyr(),
         config_.graph_builder_config().factor_noise().zero_rot_bias_velocity_noise().gyr())
            .finished());

    auto zero_rot_bias_velocity_factor = ZeroRotBiasVelocityFactor(
        frame.state.get_pose_key(), frame.state.get_bias_key(), imu->acceleration, imu->angle_rate, bias_noise);
    factor_graph.add(zero_rot_bias_velocity_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_body_velocity_factor(const LIOFrame& frame, const WheelSpeedMsg::Ptr& wheel_speed,
                                               const Eigen::Isometry3d&     T_body_imu,
                                               gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto wheel_speed_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(3) << config_.graph_builder_config().factor_noise().body_velocity_noise().x(),
         config_.graph_builder_config().factor_noise().body_velocity_noise().y(),
         config_.graph_builder_config().factor_noise().body_velocity_noise().z())
            .finished());

    Eigen::Vector3d wheel_speed_vehicle{wheel_speed->vehicle_speed, 0, 0};

    auto body_velocity_factor =
        BodyVelocityFactor(frame.state.get_pose_key(), frame.state.get_velocity_key(),
                           frame.state.get_wheel_scale_key(), wheel_speed_vehicle, T_body_imu, wheel_speed_noise);
    factor_graph.add(body_velocity_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_wheel_scale_factor(const LIOFrame& old_frame, const LIOFrame& new_frame,
                                             gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(1) << config_.graph_builder_config().factor_noise().wheel_scale_noise()).finished());
    auto wheel_scale_factor = gtsam::BetweenFactor<gtsam::Vector1>(
        old_frame.state.get_wheel_scale_key(), new_frame.state.get_wheel_scale_key(), gtsam::Vector1::Zero(), noise);
    factor_graph.add(wheel_scale_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_prior_pose_factor(const LIOFrame& frame, gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto prior_pose_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << config_.graph_builder_config().factor_noise().prior_pose_noise().r_x(),
         config_.graph_builder_config().factor_noise().prior_pose_noise().r_y(),
         config_.graph_builder_config().factor_noise().prior_pose_noise().r_z(),
         config_.graph_builder_config().factor_noise().prior_pose_noise().t_x(),
         config_.graph_builder_config().factor_noise().prior_pose_noise().t_y(),
         config_.graph_builder_config().factor_noise().prior_pose_noise().t_z())
            .finished());  // rad*rad, meter*meter

    auto prior_pose_factor = gtsam::PriorFactor<gtsam::Pose3>(frame.state.get_pose_key(),
                                                              frame.state.get_nav_state().pose(), prior_pose_noise);
    factor_graph.add(prior_pose_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_prior_imu_bias_factor(const LIOFrame& frame, gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto prior_bias_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, config_.graph_builder_config().factor_noise().prior_imu_bias_noise());
    auto prior_imu_bias_factor = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        frame.state.get_bias_key(), frame.state.get_imu_bias(), prior_bias_noise);
    factor_graph.add(prior_imu_bias_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_prior_velocity_factor(const LIOFrame& frame, gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto prior_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
        3,
        config_.graph_builder_config().factor_noise().prior_velocity_noise());  // m/s

    auto prior_velocity_factor = gtsam::PriorFactor<gtsam::Velocity3>(
        frame.state.get_velocity_key(), frame.state.get_nav_state().v(), prior_velocity_noise);
    factor_graph.add(prior_velocity_factor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_prior_wheel_scale_factor(const LIOFrame&              frame,
                                                   gtsam::NonlinearFactorGraph& factor_graph) const
{
    auto prior_wheel_scale_noise =
        gtsam::noiseModel::Isotropic::Sigma(1, config_.graph_builder_config().factor_noise().prior_wheel_scale_noise());

    auto prior_wheel_scale_factor = gtsam::PriorFactor<gtsam::Vector1>(
        frame.state.get_wheel_scale_key(), frame.state.get_wheel_scale(), prior_wheel_scale_noise);

    factor_graph.add(prior_wheel_scale_factor);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool GraphBuilder::create_odom_constrain_factor(const LIOFrame& frame_source, const LIOFrame& frame_target,
                                                const double rotation_noise, const double translation_noise,
                                                double& invalid_registration_ratio, gtsam::Pose3& pose_between_out,
                                                gtsam::NonlinearFactorGraph& factor_graph) const
{
    LidarOdometry::Result result;
    result           = create_odom_constrain(frame_source, frame_target);
    pose_between_out = result.pose_between;

    static int all_cnt     = 0;
    static int invalid_cnt = 0;

    if (result.fitness_score < config_.registration_config().thresh_fitness_score() &&
        result.overlap < config_.registration_config().thresh_overlap() &&
        result.diff_angle > config_.registration_config().thresh_diff_angle_graph_build() &&
        result.diff_translation > config_.registration_config().thresh_diff_translation_graph_build())
    {
        invalid_cnt++;
        LOG_ERROR << "fitness_score or overlap NOT satisfy threshold, fitness_score: " << result.fitness_score
                  << " ,overlap: " << result.overlap << " ,diff_angle:" << result.diff_angle
                  << " ,diff_translation: " << result.diff_translation
                  << " threshold [ fitness_score: " << config_.registration_config().thresh_fitness_score()
                  << " ,overlap: " << config_.registration_config().thresh_overlap()
                  << " ,diff_angle: " << config_.registration_config().thresh_diff_angle_graph_build()
                  << " ,diff_translation: " << config_.registration_config().thresh_diff_translation_graph_build()
                  << " ]";
        return false;
    }

    create_pose_between_factor(frame_source, frame_target, rotation_noise, translation_noise, result.pose_between,
                               factor_graph);

    all_cnt++;
    invalid_registration_ratio = static_cast<double>(invalid_cnt) / static_cast<double>(all_cnt);

    // std::cout << "fitness_score: " << result.fitness_score
    //           << " ,overlap: " << result.overlap << " ,invalid_registration_ratio: " << invalid_registration_ratio
    //           << std::endl;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GraphBuilder::create_pose_between_factor(const LIOFrame& frame_source, const LIOFrame& frame_target,
                                              const double rotation_noise, const double translation_noise,
                                              const gtsam::Pose3&          pose_between,
                                              gtsam::NonlinearFactorGraph& factor_graph) const
{
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << rotation_noise, rotation_noise, rotation_noise,
                                                translation_noise, translation_noise, translation_noise)
                                                   .finished());  // rad*rad, meter*meter (128_lidar 1e-4 1e-4 1e-4)

    auto pose_between_factor = gtsam::BetweenFactor<gtsam::Pose3>(
        frame_target.state.get_pose_key(), frame_source.state.get_pose_key(), pose_between, odometry_noise);

    factor_graph.add(pose_between_factor);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
LidarOdometry::Result GraphBuilder::create_odom_constrain(const LIOFrame& frame_source,
                                                          const LIOFrame& frame_target) const
{
    return lidar_odometry_->compute(frame_source, frame_target);
}

}  // namespace parking_slam
