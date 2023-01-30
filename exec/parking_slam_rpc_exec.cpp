#include "parking_slam_rpc_exec.h"

#include "comm/comm.h"
#include "common/type_converter.h"

namespace parking_slam
{

REGISTER_EXECUTER(ParkingSLAMRpcExec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void state_machine_callback(const std::shared_ptr<parking_slam::rpc::AVPReq>& req,
                            std::shared_ptr<parking_slam::rpc::AVPRsp>&       rsp)
{
    rsp = std::make_shared<parking_slam::rpc::AVPRsp>();

    if (req)
    {
        if (req->req() == parking_slam::rpc::Req::MAPPING)
        {
            DataCenter::get_instance()->state_machine().set_mode(parking_slam::Mode::MAPPING);
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::START);
            LOG_WARNING << "req: MAPPING";
        }

        if (req->req() == parking_slam::rpc::Req::LOCALIZATION)
        {
            DataCenter::get_instance()->state_machine().set_mode(parking_slam::Mode::LOCALIZATION);
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::START);
            LOG_WARNING << "req: LOCALIZATION";
        }

        if (req->req() == parking_slam::rpc::Req::STOP)
        {
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::STOP);
            LOG_WARNING << "req: STOP";
        }

        if (req->req() == parking_slam::rpc::Req::RESET)
        {
            DataCenter::get_instance()->state_machine().set_status(StateMachine::Status::RESET);
            LOG_WARNING << "req: RESET";
        }

        if (req->req() == parking_slam::rpc::Req::SAVE)
        {
            DataCenter::get_instance()->save_map = true;
            LOG_WARNING << "req: SAVE";
        }
        rsp->set_rsp(parking_slam::rpc::Rsp::OK);
    }
    else
    {
        rsp->set_rsp(parking_slam::rpc::Rsp::ERROR);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParkingSLAMRpcExec::ParkingSLAMRpcExec(const std::string& name, const std::string& type) : IExec(name, type)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ParkingSLAMRpcExec::init_rpc()
{
    if (DataCenter::get_instance()->config_inited())
    {
        client_ptr_ = std::make_shared<haomo::pos::RpcClient<parking_slam::rpc::AVPResult, parking_slam::rpc::AVPRsp>>(
            "parking_slam_viz");
        service_ptr_ = std::make_shared<haomo::pos::RpcServer<parking_slam::rpc::AVPReq, parking_slam::rpc::AVPRsp>>(
            "parking_slam_state_machine", state_machine_callback);

        client_ptr_->set_timeout(0, 10000);

        init_ = true;
    }

    return init_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParkingSLAMRpcExec::process(const ::haomo::pos::IData* const input, ::haomo::pos::IData* const output)
{
    if (!init_)
    {
        init_rpc();
    }

    if (!init_)
    {
        return 0;
    }

    const auto predict_states = DataCenter::get_instance()->get_predict_state_deque();

    if (predict_states.empty())
    {
        return 0;
    }

    Eigen::Isometry3d T_imu_body = DataCenter::get_instance()->hardware_config.get_rtk_T_body();

    auto avp_result_ptr = std::make_shared<parking_slam::rpc::AVPResult>();
    avp_result_ptr->set_timestamp(predict_states.front().get_timestamp().second());

    for (size_t i = 0; i < predict_states.size(); ++i)
    {
        const auto&       state         = predict_states[i];
        Eigen::Isometry3d T_world_imu   = Eigen::Isometry3d::Identity();
        T_world_imu.linear()            = state.get_nav_state().pose().rotation().matrix();
        T_world_imu.translation()       = state.get_nav_state().pose().translation();
        Eigen::Isometry3d  T_world_body = T_world_imu * T_imu_body;
        Eigen::Quaterniond q_world_body(T_world_body.linear());
        auto               pose = avp_result_ptr->add_trajectory();

        pose->set_timestamp(state.get_timestamp().second());
        pose->set_rtk_status(state.get_rtk_status());

        pose->mutable_rotation()->set_x(q_world_body.x());
        pose->mutable_rotation()->set_y(q_world_body.y());
        pose->mutable_rotation()->set_z(q_world_body.z());
        pose->mutable_rotation()->set_w(q_world_body.w());

        pose->mutable_translation()->set_x(T_world_body.translation().x());
        pose->mutable_translation()->set_y(T_world_body.translation().y());
        pose->mutable_translation()->set_z(T_world_body.translation().z());
    }

    auto rsp = client_ptr_->request(avp_result_ptr);

    return 0;
}

}  // namespace parking_slam
