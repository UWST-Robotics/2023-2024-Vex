#pragma once
#include "pros/rtos.hpp"
#include "devils/devils.h"

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(initialPath, "paths/pj-auto.txt");
INCTXT(finishPath, "paths/pj-finish.txt");
INCTXT(occupancyGrid, "paths/occupancy.txt");

namespace devils
{
    /**
     * Runs the Autonomous for PJ
     */
    class PJAutoController : public AutoController
    {
    public:
        PJAutoController(
            BaseChassis &chassis,
            OdomSource &odometry,
            GameObjectManager &gameObjectManager)
            : odometry(odometry),
              initialController(chassis, odometry, &initialPath),
              collectionController(chassis, odometry, gameObjectManager, occupancyGrid),
              reverseControllerA(chassis, REVERSE_DURATION, REVERSE_SPEED),
              goalController(chassis, odometry, occupancyGrid),
              pushController(chassis, PUSH_DURATION, PUSH_SPEED),
              reverseControllerB(chassis, REVERSE_DURATION, REVERSE_SPEED),
              returnController(chassis, odometry, occupancyGrid),
              autoPointController(chassis, odometry, &finishPath)
        {
            returnController.setTargetPose(*finishPath.getStartingPose());
            reset();
        }

        void reset() override
        {
            mainStack.reset();
        }

        void update() override
        {
            auto currentController = mainStack.getCurrentController(true);

            // Update w/ Closest Goal Pose
            if (currentController == &goalController)
                goalController.setTargetPose(_getClosestGoalPose());

            // Update Path Renderer
            if (currentController == &initialController && pathRenderer != nullptr)
                pathRenderer->setPath(initialPath);
            if (currentController == &autoPointController && pathRenderer != nullptr)
                pathRenderer->setPath(finishPath);

            // Update State
            currentState.events = currentController->getState().events;
            mainStack.update();
        }

        /**
         * Only grabs the game objects in the collection area
         * @param polygon The collection polygon
         */
        void useCollectionArea(Polygon *polygon)
        {
            collectionController.useCollectionArea(polygon);
        }

        /**
         * Updates the path renderer on path generation
         * @param renderer The path renderer
         */
        void usePathRenderer(PathRenderer *renderer)
        {
            pathRenderer = renderer;
            collectionController.usePathRenderer(renderer);
            goalController.usePathRenderer(renderer);
            returnController.usePathRenderer(renderer);
        }

        /**
         * Sets the vision sensor for game object collection
         * @param sensor The vision sensor
         */
        void useVisionSensor(VisionSensor *sensor)
        {
            collectionController.useVisionSensor(sensor);
        }

        /**
         * Gets the starting pose of the robot
         * @return The starting pose
         */
        Pose *getStartingPose()
        {
            return initialPath.getStartingPose();
        }

        /**
         * Gets the closest pose from the robot to the goal
         * @return The closest goal pose
         */
        Pose _getClosestGoalPose()
        {
            auto currentPose = odometry.getPose();
            Pose closestPose = GOAL_POSES[0];
            double closestDistance = currentPose.distanceTo(closestPose);

            for (auto &pose : GOAL_POSES)
            {
                double distance = currentPose.distanceTo(pose);
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPose = pose;
                }
            }

            return closestPose;
        }

    private:
        static constexpr double CHASSIS_SPEED = 0.5;
        static constexpr int REVERSE_DURATION = 300; // ms
        static constexpr double REVERSE_SPEED = -0.5;
        static constexpr int PUSH_DURATION = 100; // ms
        static constexpr double PUSH_SPEED = 1.0;
        static constexpr int AUTO_POINT_TIMEOUT = 30 * 1000; // 30 seconds (15 seconds left for auto point)
        const std::vector<Pose> GOAL_POSES = {
            Pose(60, 24),
            Pose(48, 10),
            Pose(48, -10),
            Pose(60, -24)};

        // Systems
        OdomSource &odometry;

        // Paths
        OccupancyGrid occupancyGrid = OccupancyFileReader::deserialize(g_occupancyGridData);
        GeneratedPath initialPath = PathGenerator::generateSpline(PathFileReader::deserialize(g_initialPathData));
        GeneratedPath finishPath = PathGenerator::generateSpline(PathFileReader::deserialize(g_finishPathData));

        // Controllers
        PursuitController initialController;       // Initial Path
        CollectionController collectionController; // Game Object Collection
        TimeController reverseControllerA;         // Reverse A
        FindController goalController;             // Return to Goal
        TimeController pushController;             // Push Goal
        TimeController reverseControllerB;         // Reverse B
        FindController returnController;           // Drive to Auto Point
        PursuitController autoPointController;     // Auto Point

        // Controller Stacks
        ControllerList loopStack = ControllerList({&collectionController, &reverseControllerA, &goalController, /*&pushController,*/ &reverseControllerB}, true);
        ControllerList timeoutStack = ControllerList({&initialController, &loopStack}, false, AUTO_POINT_TIMEOUT);
        ControllerList mainStack = ControllerList({&timeoutStack, &returnController, &autoPointController});

        /*
            1. Initial Path
            
            2. Loop:
                a. Collection
                b. Reverse
                c. Goal
                d. Push
                e. Reverse
            
            3. Return
            4. Auto Point
        */

        // Optional Components
        PathRenderer *pathRenderer = nullptr;
    };
}