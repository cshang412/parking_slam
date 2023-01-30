#pragma once

#include <iostream>
#include <typeinfo>

#include "data.h"
#include "data_center/data_center.h"
#include "map_manager.h"

namespace parking_slam
{

/**
 * @brief
 *
 */
class MapManagerExec : public ::haomo::pos::IExec
{
public:
    /**
     * @brief Construct a new MapManager Exec object
     *
     * @param name
     * @param type
     */
    MapManagerExec(const std::string& name, const std::string& type);

    /**
     * @brief Destroy the Map Manager Exec object
     *
     */
    ~MapManagerExec()
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

        // stop_   = false;
        // thread_ = std::thread(std::bind(&MapManagerExec::run, this));
        // thread_.detach();

        return 0;
    }

    /**
     * @brief
     *
     * @return int
     */
    int release()
    {
        // stop_ = true;

        // if (thread_.joinable())
        // {
        //     thread_.join();
        // }

        LOG_WARNING << "In MapManagerExec release";

        return 0;
    }

private:
    // /**
    //  * @brief
    //  *
    //  */
    void process_save_map();

    /**
     * @brief
     *
     */
    // std::atomic<bool> stop_;
    // std::thread       thread_;
    bool              localization_map_inited_ = false;
    
    std::unordered_map<int, haomo::hidelivery::perception::ParkingSlot3D> slot_map_;
};
}  // namespace parking_slam
