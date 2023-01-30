#pragma once
#include <iostream>
#include <typeinfo>

#include "data.h"
#include "data_center/data_center.h"

namespace parking_slam
{
/**
 * @brief
 *
 */
class LIDARReceiveExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new LIDARReceiveExec object
     *
     * @param name
     * @param type
     */
    LIDARReceiveExec(const std::string& name, const std::string& type) : IExec(name, type)
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

        return 0;
    }

    /**
     * @brief
     *
     * @return int
     */
    int release()
    {
        return 0;
    }

private:
    /**
     * @brief
     *
     * @param p_bundle
     * @param msg_name
     * @return parking_slam::PointCloudMsg::Ptr
     */
    parking_slam::PointCloudMsg::Ptr parse_point_cloud_msg(const ::haomo::pos::BundleData* p_bundle,
                                                          const std::string&              msg_name) const;
};
}  // namespace parking_slam
