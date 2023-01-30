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
class PoseManagerExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new PoseManagerExec object
     *
     * @param name
     * @param type
     */
    PoseManagerExec(const std::string& name, const std::string& type) : IExec(name, type)
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
};
}  // namespace parking_slam
