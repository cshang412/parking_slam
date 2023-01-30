#pragma once
#include <iostream>
#include <typeinfo>

#include "avp_rpc.pb.h"
#include "comm/comm.h"
#include "data.h"
#include "data_center/data_center.h"
#include "lio_frame.h"

namespace parking_slam
{

/**
 * @brief
 *
 */
class ParkingSLAMRpcExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new Visualizer Exec object
     *
     * @param name
     * @param type
     */
    ParkingSLAMRpcExec(const std::string& name, const std::string& type);

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
        LOG_WARNING << "In ParkingSLAMRpcExec release";
        return 0;
    }

private:
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool init_rpc();

private:
    bool init_ = false;
    /**
     * @brief
     *
     */
    std::shared_ptr<haomo::pos::RpcClient<parking_slam::rpc::AVPResult, parking_slam::rpc::AVPRsp> > client_ptr_;
    std::shared_ptr<haomo::pos::RpcServer<parking_slam::rpc::AVPReq, parking_slam::rpc::AVPRsp> >    service_ptr_;
};
}  // namespace parking_slam
