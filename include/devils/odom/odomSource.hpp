#pragma once
#include "pose.hpp"

namespace devils
{
    struct OdomSource
    {
    public:
        virtual const Pose getPose() { return Pose(); }
        virtual void setPose(const Pose pose) {}
    };
}