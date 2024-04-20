#pragma once
#include "devils/devils.h"

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(mainPath, "paths/test.txt");

namespace devils
{

    /**
     * Controls the test controller.
     */
    class TestController : public ControllerList
    {
    public:
        TestController(
            BaseChassis &chassis,
            OdomSource &odometry)
            : pursuitController(chassis, odometry, &mainPath),
              ControllerList({&pursuitController}, true)
        {
        }

        Pose *getStartingPose()
        {
            return mainPath.getStartingPose();
        }

        void usePathRenderer(PathRenderer &renderer)
        {
            renderer.setPath(mainPath);
        }

    private:
        // Path
        GeneratedPath mainPath = PathGenerator::generateSpline(PathFileReader::deserialize(g_mainPathData));

        // Controllers
        PursuitController pursuitController;
    };
}