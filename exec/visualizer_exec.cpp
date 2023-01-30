#include "visualizer_exec.h"

#include "common/type_converter.h"

namespace parking_slam
{

REGISTER_EXECUTER(VisualizerExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
VisualizerExec::VisualizerExec(const std::string& name, const std::string& type) : IExec(name, type)
{
    T_imu_body_ = Eigen::Isometry3d::Identity();

    pangolin::CreateWindowAndBind("Parking SLAM Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    s_cam_ = std::shared_ptr<pangolin::OpenGlRenderState>(
        new pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                        ///! camera position, model position, upper vector
                                        pangolin::ModelViewLookAt(-2, 0, 0, 0, 0, 0, pangolin::AxisZ)));

    handler_ = std::make_shared<pangolin::Handler3D>(*s_cam_);

    ///! display point cloud
    d_cam_ = &pangolin::CreateDisplay().SetBounds(1 / 3, 1, 0.0, 1.0).SetHandler(handler_.get());

    pangolin::CreatePanel("menu").SetBounds(0.7, 1, 0.0, 0.20);
    follow_ = std::make_shared<pangolin::Var<bool>>("menu.Follow", true, true);
    rotate_ = std::make_shared<pangolin::Var<bool>>("menu.Rotate", false, true);
    zoom_   = std::make_shared<pangolin::Var<int>>("menu.Zoom", 100, 20, 200);

    draw_map_          = std::make_shared<pangolin::Var<bool>>("menu.Map", true, true);
    draw_current_scan_ = std::make_shared<pangolin::Var<bool>>("menu.Scan", true, true);
    draw_trajectory_   = std::make_shared<pangolin::Var<bool>>("menu.Trajectory", true, true);
    draw_ref_line_     = std::make_shared<pangolin::Var<bool>>("menu.Ref LIne", true, true);

    pangolin::GetBoundWindow()->RemoveCurrent();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int VisualizerExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    pangolin::BindToContext("Parking SLAM Viewer");
    glEnable(GL_DEPTH_TEST);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam_->Activate(*s_cam_);

    if (!DataCenter::get_instance()->inited() || !DataCenter::get_instance()->lio_inited())
    {
        pangolin::FinishFrame();
        pangolin::GetBoundWindow()->RemoveCurrent();

        return 0;
    }

    const auto key_frame_deque = DataCenter::get_instance()->get_current_key_frame_deque();
    const auto predict_states  = DataCenter::get_instance()->get_predict_state_deque();
    const auto ref_line        = DataCenter::get_instance()->get_ref_line();

    if (predict_states.empty() || key_frame_deque.empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        return 0;
    }

    static std::unordered_map<int, Eigen::Vector3f> map_color_map;

    if (draw_map_->Get() && DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::LOCALIZATION)
    {
        // do something

        if (DataCenter::get_instance()->map.map.save_ptr)
        {
            glPointSize(3);
            glBegin(GL_POINTS);
            for (std::uint32_t i = 0; i < DataCenter::get_instance()->color_map.size(); i += 5)
            {
                const auto& p = DataCenter::get_instance()->color_map.points[i];
                glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
                glVertex3d(p.x, p.y, p.z);
            }
            glEnd();
        }

        if (!key_frame_deque.empty() && key_frame_deque.back().save_ptr)
        {
            auto              kf   = key_frame_deque.back();
            Eigen::Isometry3d T_wi = parking_slam::TypeConverter::nav_state_to_iso3d(kf.state.get_nav_state());
            Eigen::Isometry3d T_wb = T_wi * DataCenter::get_instance()->hardware_config.get_rtk_T_body();
            Eigen::Matrix4f   T_wf = T_wb.matrix().template cast<float>();
            pangolin::glSetFrameOfReference(T_wf);
            glPointSize(5);
            glBegin(GL_POINTS);

            for (std::uint32_t i = 0; i < kf.save_ptr->points.size(); i += 1)
            {
                const auto& p = kf.save_ptr->points[i];

                glColor3d(1.0, 0.0, 0.0);
                glVertex3d(p.x, p.y, p.z);
            }
            glEnd();
            pangolin::glUnsetFrameOfReference();
        }
    }

    if (draw_map_->Get() && DataCenter::get_instance()->state_machine().get_mode() == parking_slam::Mode::MAPPING)
    {
        // do something
        for (const auto& kf : key_frame_deque)
        {
            Eigen::Isometry3d T_wi = parking_slam::TypeConverter::nav_state_to_iso3d(kf.state.get_nav_state());
            Eigen::Isometry3d T_wb = T_wi * DataCenter::get_instance()->hardware_config.get_rtk_T_body();
            Eigen::Matrix4f   T_wf = T_wb.matrix().template cast<float>();
            pangolin::glSetFrameOfReference(T_wf);
            glPointSize(3);
            glBegin(GL_POINTS);

            for (std::uint32_t i = 0; i < kf.save_ptr->points.size(); i += 5)
            {
                const auto& p = kf.save_ptr->points[i];

                int             z_key = (p.z / 0.01);
                Eigen::Vector3f color;

                if (map_color_map.count(z_key))
                {
                    color = map_color_map[z_key];
                }
                else
                {
                    float r, g, b;
                    scalar_to_color(p.z, r, g, b);
                    color = Eigen::Vector3f(r, g, b);
                    map_color_map.insert({z_key, color});
                }

                glColor3d(color.x(), color.y(), color.z());
                glVertex3d(p.x, p.y, p.z);
            }
            glEnd();
            pangolin::glUnsetFrameOfReference();
        }
    }

    if (draw_trajectory_->Get())
    {
        glPointSize(5);
        glBegin(GL_POINTS);

        for (std::uint32_t i = 0; i < predict_states.size(); i++)
        {
            glColor3d(1.0, 0.0, 0.0);
            Eigen::Isometry3d T_world_imu =
                parking_slam::TypeConverter::nav_state_to_iso3d(predict_states[i].get_nav_state());
            Eigen::Isometry3d T_world_body = T_world_imu * DataCenter::get_instance()->hardware_config.get_rtk_T_body();
            Eigen::Vector3d   translation  = T_world_body.translation();
            glVertex3d(translation.x(), translation.y(), translation.z() + 0.5);
        }
        glEnd();
    }

    if(draw_ref_line_->Get())
    {
        glPointSize(3);
        glBegin(GL_POINTS);

        for (std::uint32_t i = 0; i < ref_line.size(); i++)
        {
            glColor3d(1.0, 0.0, 0.0);
            glVertex3d(ref_line[i].x(), ref_line[i].y(), ref_line[i].z());
        }
        glEnd();
    }

    if (follow_->Get() && !key_frame_deque.empty())
    {
        Eigen::Isometry3d T_wi =
            parking_slam::TypeConverter::nav_state_to_iso3d(key_frame_deque.back().state.get_nav_state());
        Eigen::Isometry3d T_w_body = T_wi * DataCenter::get_instance()->hardware_config.get_rtk_T_body();

        pangolin::OpenGlMatrix mv;
        Eigen::Vector3d        forward_vector(0, 0, -1);
        Eigen::Vector3d        up_vector(1, 0, 0);

        static const Eigen::Matrix3d boot_R = T_w_body.linear();

        Eigen::Vector3d forward = T_w_body.linear() * forward_vector;
        Eigen::Vector3d up      = T_w_body.linear() * up_vector;

        if (!rotate_->Get())
        {
            forward = (boot_R * forward_vector);
            up      = (boot_R * up_vector);
        }

        Eigen::Vector3d eye =
            T_w_body.translation() + Eigen::Vector3d{20, 0, static_cast<double>(zoom_->Get())} - forward;
        Eigen::Vector3d at = eye + forward;

        Eigen::Vector3d z = (eye - at).normalized();   // Forward
        Eigen::Vector3d x = up.cross(z).normalized();  // Right
        Eigen::Vector3d y = z.cross(x);

        Eigen::Matrix4d m;
        m << x(0), x(1), x(2), -(x.dot(eye)), y(0), y(1), y(2), -(y.dot(eye)), z(0), z(1), z(2), -(z.dot(eye)), 0, 0, 0,
            1;

        memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));
        s_cam_->SetModelViewMatrix(mv);
    }

    pangolin::FinishFrame();
    pangolin::GetBoundWindow()->RemoveCurrent();

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VisualizerExec::scalar_to_color(const float& value, float& r, float& g, float& b, const float min_value,
                                     const float max_value, const int color_map_type)
{
    Eigen::Vector3f topColor;
    Eigen::Vector3f midColor;
    Eigen::Vector3f bottomColor;

    if (color_map_type == 0)
    {
        topColor    = Eigen::Vector3f(0, 255, 0);
        midColor    = Eigen::Vector3f(0, 255, 255);
        bottomColor = Eigen::Vector3f(0, 0, 255);
    }
    else
    {
        topColor    = Eigen::Vector3f(255, 0, 0);
        midColor    = Eigen::Vector3f(0, 255, 0);
        bottomColor = Eigen::Vector3f(0, 0, 255);
    }

    float mid_value = (min_value + max_value) / 2;

    // -----------------------------颜色渲染--------------------------------------
    if (value < mid_value)
    {
        float k1 = (value - min_value) / (mid_value - min_value);
        r        = (bottomColor[0] + (midColor[0] - bottomColor[0]) * k1) / 255.0;
        g        = (bottomColor[1] + (midColor[1] - bottomColor[1]) * k1) / 255.0;
        b        = (bottomColor[2] + (midColor[2] - bottomColor[2]) * k1) / 255.0;
    }
    else
    {
        float k2 = (value - min_value) / (max_value - mid_value);
        r        = (midColor[0] + (topColor[0] - midColor[0]) * k2) / 255.0;
        g        = (midColor[1] + (topColor[1] - midColor[1]) * k2) / 255.0;
        b        = (midColor[2] + (topColor[2] - midColor[2]) * k2) / 255.0;
    }
}

}  // namespace parking_slam
