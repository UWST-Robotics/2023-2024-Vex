#pragma once
#include "devils/devils.h"

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(blazePath, "paths/blaze-auto.txt");

namespace devils
{

    /**
     * Controls the autonomous for Blaze.
     */
    class BlazeAutoController : public ControllerList
    {
    public:
        BlazeAutoController(
            BaseChassis &chassis,
            OdomSource &odometry)
            : pursuitController(chassis, odometry, &mainPath),
              ControllerList({&pursuitController}, false)
        {
            pursuitController.setLookaheadDistance(6);
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
        GeneratedPath mainPath = PathGenerator::generateSpline(PathFileReader::deserialize(g_blazePathData));

        // Controllers
        PursuitController pursuitController;
    };
}