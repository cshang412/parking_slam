#include <pangolin/pangolin.h>

#include <chrono>
#include <iomanip>
#include <string>
#include <thread>

#include "avp_rpc.pb.h"
#include "comm/comm.h"
#include "data.h"
#include "data_provider/dataset.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
const std::string              config_path = "/home/ubuntu/haomo/data/config.prototxt";
std::vector<Eigen::Isometry3d> traj_data_out;
std::mutex                     mutex_traj_data;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void test_service(const std::shared_ptr<parking_slam::rpc::AVPResult>& req,
                  std::shared_ptr<parking_slam::rpc::AVPRsp>&          rsp)
{
    if (rsp == nullptr)
    {
        rsp = std::make_shared<parking_slam::rpc::AVPRsp>();
    }

    if (req == nullptr)
    {
        rsp->set_rsp(parking_slam::rpc::Rsp::ERROR);

        return;
    }
    std::vector<Eigen::Isometry3d> traj_data;
    for (int i = 0; i < req->trajectory_size(); ++i)
    {
        Eigen::Vector3d    t_world_body;
        Eigen::Quaterniond q_world_body;

        q_world_body.w() = req->trajectory(i).rotation().w();
        q_world_body.x() = req->trajectory(i).rotation().x();
        q_world_body.y() = req->trajectory(i).rotation().y();
        q_world_body.z() = req->trajectory(i).rotation().z();

        t_world_body.x() = req->trajectory(i).translation().x();
        t_world_body.y() = req->trajectory(i).translation().y();
        t_world_body.z() = req->trajectory(i).translation().z();

        Eigen::Isometry3d pose;
        pose.linear()      = q_world_body.toRotationMatrix();
        pose.translation() = t_world_body;

        traj_data.push_back(pose);
    }

    {
        std::lock_guard<std::mutex> lg(mutex_traj_data);
        traj_data_out = traj_data;
    }

    rsp->set_rsp(parking_slam::rpc::Rsp::OK);

    return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_rsp(const std::shared_ptr<parking_slam::rpc::AVPRsp>& rsp)
{
    if (rsp != nullptr)
    {
        std::cout << "RPC rsq successfully !" << std::endl;
    }
    else
    {
        std::cout << "RPC rsq error !!! " << std::endl;
    }
}

DEFINE_string(log_path, "/home/ubuntu/haomo/parking_slam/log", "log_path");
DEFINE_string(config_path, "/home/ubuntu/haomo/parking_slam/modules/parking_slam/config/arch_x86/config.prototxt", "log_path");

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    parking_slam::ParkingSLAMConfig config;
    ;
    if (!::haomo::pos::read_proto_from_text_file(FLAGS_config_path.c_str(), &config))
    {
        std::cerr << "load config file error!" << std::endl;
        return -1;
    }

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "log_path: " << FLAGS_log_path << std::endl;
    std::cout << "rpc_ip: " << config.rpc_config().rpc_ip() << std::endl;
    haomo::pos::log_init(argv[0], FLAGS_log_path.c_str(), ERROR);
    // 设置IP， 当作为对立进程使用comm时，必须设置一次IP
    os::SysInfo::instance()->ip(config.rpc_config().rpc_ip());
    haomo::pos::Transport::init();

    // RPC server
    haomo::pos::RpcServer<parking_slam::rpc::AVPResult, parking_slam::rpc::AVPRsp> service("parking_slam_viz",
                                                                                           test_service);
    haomo::pos::RpcClient<parking_slam::rpc::AVPReq, parking_slam::rpc::AVPRsp>    client("parking_slam_state_machine");
    client.set_timeout(0, 10000);

    pangolin::CreateWindowAndBind("Parking Carui", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    ///! projection view
    pangolin::OpenGlRenderState s_cam =
        (pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                     ///! camera position, model position, upper vector
                                     pangolin::ModelViewLookAt(0, 0, -2, 0, 0, 0, pangolin::AxisNegY)));

    ///! for 3d interaction
    pangolin::Handler3D handler(s_cam);

    ///! display point cloud
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(1 / 3, 1, 0.0, 1.0).SetHandler(&handler);

    pangolin::CreatePanel("menu").SetBounds(0.6, 1, 0.0, 0.20);
    pangolin::Var<bool> follow("menu.Follow", true, true);
    pangolin::Var<int>  zoom("menu.Zoom", 100, 20, 200);
    pangolin::Var<bool> rotate("menu.Rotate", false, true);
    pangolin::Var<bool> mapping("menu.mapping", true, false);
    pangolin::Var<bool> localization("menu.localization", true, false);
    pangolin::Var<bool> stop("menu.stop", true, false);
    pangolin::Var<bool> reset("menu.reset", true, false);
    pangolin::Var<bool> save("menu.save", true, false);

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        auto req = std::make_shared<parking_slam::rpc::AVPReq>();

        if (pangolin::Pushed(mapping))
        {
            req->set_req(parking_slam::rpc::Req::MAPPING);
            process_rsp(client.request(req));

            std::cout << "RPC req: Mapping " << std::endl;
        }

        if (pangolin::Pushed(localization))
        {
            req->set_req(parking_slam::rpc::Req::LOCALIZATION);
            process_rsp(client.request(req));

            std::cout << "RPC req: Localization " << std::endl;
        }

        if (pangolin::Pushed(stop))
        {
            req->set_req(parking_slam::rpc::Req::STOP);
            process_rsp(client.request(req));

            std::cout << "RPC req: Stop " << std::endl;
        }

        if (pangolin::Pushed(reset))
        {
            req->set_req(parking_slam::rpc::Req::RESET);
            process_rsp(client.request(req));

            std::cout << "RPC req: Reset " << std::endl;
        }

        if (pangolin::Pushed(save))
        {
            req->set_req(parking_slam::rpc::Req::SAVE);
            process_rsp(client.request(req));

            std::cout << "RPC req: Save " << std::endl;
        }

        std::vector<Eigen::Isometry3d> td;

        {
            std::lock_guard<std::mutex> lg(mutex_traj_data);
            td = traj_data_out;
        }

        {
            glPointSize(3);
            glBegin(GL_POINTS);

            for (std::uint32_t i = 0; i < td.size(); i++)
            {
                glColor3d(0.0, 1.0, 0.0);
                Eigen::Vector3d translation = td[i].translation();
                glVertex3d(translation.x(), translation.y(), translation.z());
            }
            glEnd();
        }

        if (follow.Get() && !td.empty())
        {
            Eigen::Isometry3d T_w_body = td.back();

            pangolin::OpenGlMatrix mv;
            Eigen::Vector3d        forward_vector(0, 0, -1);
            Eigen::Vector3d        up_vector(1, 0, 0);

            static const Eigen::Matrix3d boot_R = T_w_body.linear();

            Eigen::Vector3d forward = T_w_body.linear() * forward_vector;
            Eigen::Vector3d up      = T_w_body.linear() * up_vector;

            if (!rotate.Get())
            {
                forward = (boot_R * forward_vector);
                up      = (boot_R * up_vector);
            }

            Eigen::Vector3d eye =
                T_w_body.translation() + Eigen::Vector3d{20, 0, static_cast<double>(zoom.Get())} - forward;
            Eigen::Vector3d at = eye + forward;

            Eigen::Vector3d z = (eye - at).normalized();   // Forward
            Eigen::Vector3d x = up.cross(z).normalized();  // Right
            Eigen::Vector3d y = z.cross(x);

            Eigen::Matrix4d m;
            m << x(0), x(1), x(2), -(x.dot(eye)), y(0), y(1), y(2), -(y.dot(eye)), z(0), z(1), z(2), -(z.dot(eye)), 0,
                0, 0, 1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));
            s_cam.SetModelViewMatrix(mv);
        }

        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    haomo::pos::log_release();

    return 0;
}
