#pragma once
#include <pangolin/pangolin.h>

#include <iostream>
#include <typeinfo>

#include "data.h"
#include "data_center/data_center.h"
#include "lio_frame.h"

namespace parking_slam
{

/**
 * @brief
 *
 */
class VisualizerExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new Visualizer Exec object
     *
     * @param name
     * @param type
     */
    VisualizerExec(const std::string& name, const std::string& type);

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
        LOG_WARNING << "In VisualizerExec release";
        return 0;
    }

private:
    void scalar_to_color(const float& value, float& r, float& g, float& b, const float min_value = -0.5,
                         const float max_value = 4.0, const int color_map_type = 0);

private:
    /**
     * @brief
     *
     */
    Eigen::Isometry3d T_imu_body_;

    std::shared_ptr<pangolin::OpenGlRenderState> s_cam_;
    std::shared_ptr<pangolin::Handler3D>         handler_;
    pangolin::View*                              d_cam_;
    std::shared_ptr<pangolin::Var<bool>>         follow_;
    std::shared_ptr<pangolin::Var<bool>>         rotate_;
    std::shared_ptr<pangolin::Var<int>>          zoom_;

    std::shared_ptr<pangolin::Var<bool>> draw_trajectory_;
    std::shared_ptr<pangolin::Var<bool>> draw_map_;
    std::shared_ptr<pangolin::Var<bool>> draw_current_scan_;
    std::shared_ptr<pangolin::Var<bool>> draw_ref_line_;

    parking_slam::ParkingSLAMConfig config_;
};
}  // namespace parking_slam
