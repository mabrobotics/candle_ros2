#pragma once

#include "pds.hpp"
#include "candle_ros2/pds_modules/base_module_ros.hpp"

struct PdsInstance
{
    std::unique_ptr<mab::Pds>                   pds;
    std::unique_ptr<BaseModuleRos>              ctrlModule;
    std::vector<std::unique_ptr<BaseModuleRos>> modules;

    PdsInstance() = default;

    PdsInstance(PdsInstance&&) noexcept            = default;
    PdsInstance& operator=(PdsInstance&&) noexcept = default;

    PdsInstance(const PdsInstance&)            = delete;
    PdsInstance& operator=(const PdsInstance&) = delete;
};
