#pragma once
/* PDS ROS2 modules */
#include "candle_ros2/pds_modules/base_module_ros.hpp"

/* CANdle-SDK */
#include "pds.hpp"
struct pdsInstance_S
{
    std::unique_ptr<mab::Pds>                     pds;
    std::unique_ptr<I_BaseModuleRos>              ctrlModule;
    std::vector<std::unique_ptr<I_BaseModuleRos>> modules;

    pdsInstance_S() = default;

    pdsInstance_S(pdsInstance_S&&) noexcept            = default;
    pdsInstance_S& operator=(pdsInstance_S&&) noexcept = default;

    pdsInstance_S(const pdsInstance_S&)            = delete;
    pdsInstance_S& operator=(const pdsInstance_S&) = delete;
};
