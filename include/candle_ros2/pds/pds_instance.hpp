#pragma once

#include "pds.hpp"
#include "candle_ros2/pds_modules/base_module_ros.hpp"

struct PdsInstance
{
    mab::Pds                                    pds;
    std::vector<std::unique_ptr<BaseModuleRos>> modules;

    explicit PdsInstance(mab::Pds&& p) : pds(std::move(p))
    {
    }

    PdsInstance(PdsInstance&&) noexcept            = default;
    PdsInstance& operator=(PdsInstance&&) noexcept = default;

    PdsInstance(const PdsInstance&)            = delete;
    PdsInstance& operator=(const PdsInstance&) = delete;
};
