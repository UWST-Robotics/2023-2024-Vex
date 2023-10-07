#pragma once
#include "pathFileReader.h"
#include "pathFile.h"
#include "okapi/api/control/util/pathfinderUtil.hpp"
#include <vector>

namespace devils
{
    class MotionProfile
    {
    public:
        void GenerateMotionProfile();

    private:
        const float PATH_MAX_VEL = 0.0f;
        const float PATH_MAX_ACCEL = 0.0f;
        const float PATH_MAX_JERK = 0.0f;

        std::vector<okapi::PathfinderPoint> getPoints(PathFile path);
    };
}