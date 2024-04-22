#pragma once
#include "devils/devils.h"

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(pjPath, "paths/pj-auto.txt");
INCTXT(pjSkills, "paths/pj-skills.txt");

namespace devils
{

    /**
     * Controls the autonomous for PJ.
     */
    class PJAutoController : public ControllerList
    {
    public:
        PJAutoController(
            BaseChassis &chassis,
            OdomSource &odometry)
            : pursuitController(chassis, odometry, &mainPath),
              ControllerList({&pursuitController}, false)
        {
        }

        /**
         * Gets the path to use.
         * @return The path to use.
         */
        GeneratedPath &_getPath()
        {
            return isSkillsPath ? skillsPath : mainPath;
        }

        Pose *getStartingPose()
        {
            return _getPath().getStartingPose();
        }

        /**
         * Uses the given path renderer.
         * @param renderer The path renderer to use.
         */
        void usePathRenderer(PathRenderer &renderer)
        {
            pathRenderer = &renderer;
            pathRenderer->setPath(_getPath());
        }

        /**
         * Enables or disables the skills path.
         * @param enable Whether to enable the skills path.
         */
        void enableSkills(bool enable)
        {
            isSkillsPath = enable;
            pursuitController.setPath(&_getPath());
            if (pathRenderer != nullptr)
                pathRenderer->setPath(_getPath());
        }

    private:
        // Path
        GeneratedPath mainPath = PathGenerator::generateSpline(PathFileReader::deserialize(g_pjPathData));
        GeneratedPath skillsPath = PathGenerator::generateSpline(PathFileReader::deserialize(g_pjSkillsData));
        bool isSkillsPath = false;

        // Controllers
        PursuitController pursuitController;
        PathRenderer *pathRenderer = nullptr;
    };
}