#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/pid.hpp"
#include "../geometry/lerp.hpp"
#include "autoController.hpp"
#include "directController.hpp"
#include <cmath>
#include <vector>

namespace devils
{
    /**
     * Controller for follwing a path with sensor feedback using Basic Pure Pursuit.
     */
    class PursuitController : public AutoController
    {
    public:
        /**
         * Constructs a new PursuitController.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         * @param path The generated path to follow.
         * @param skipCheckpoints Whether the controller can skip checkpoints.
         */
        PursuitController(BaseChassis &chassis, OdomSource &odometry, GeneratedPath *path = nullptr, bool skipCheckpoints = false)
            : chassis(chassis),
              odometry(odometry),
              directController(chassis, odometry),
              skipCheckpoints(skipCheckpoints),
              currentPath(path)
        {
            setPath(path);
        }

        void reset() override
        {
            AutoController::reset();
            directController.reset();
            robotPointIndex = 0;
            lookaheadPointIndex = 0;
            controlPointIndex = 0;
        }

        void update() override
        {
            // Abort if path is missing
            if (currentPath == nullptr || pathPoints == nullptr || controlPoints == nullptr)
                return;
            // Abort if finished
            if (currentState.isFinished)
                return;

            // Get Current Pose
            Pose currentPose = odometry.getPose();

            // Update Control Point Index
            ControlPoint *controlPoint = &controlPoints->at(controlPointIndex);
            int checkpointPathIndex = (this->controlPointIndex + 1) / currentPath->dt;

            // Update Path Point Index
            double closestDistance = INT_MAX;
            for (int i = robotPointIndex; i < pathPoints->size(); i++)
            {
                Pose point = pathPoints->at(i);
                double distance = point.distanceTo(currentPose);

                // Check if closest
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    robotPointIndex = i;
                }

                if (i > checkpointPathIndex && !skipCheckpoints)
                    break;
            }

            // Update Lookahead Point Index
            Pose *targetPose = nullptr;
            for (int i = lookaheadPointIndex; i < pathPoints->size(); i++)
            {
                targetPose = &pathPoints->at(i);
                if (targetPose->distanceTo(currentPose) > LOOKAHEAD_DISTANCE)
                    break;
                lookaheadPointIndex = i;
            }

            // Checkpoint
            bool isLookaheadPastCheckpoint = lookaheadPointIndex > checkpointPathIndex;
            bool isRotated = std::abs(Units::diffRad(currentPose.rotation, targetPose->rotation)) < ROTATIONAL_THRESHOLD;
            if (isLookaheadPastCheckpoint && isRotated)
            {
                // Increment Control Point
                if (controlPointIndex < controlPoints->size() - 1)
                {
                    controlPointIndex++;

                    // Re-run update to get new control point
                    update();
                    return;
                }
                // Finish
                else
                {
                    Logger::debug("Finished path");
                    currentState.isFinished = true;
                    chassis.stop();
                    return;
                }
            }

            // Update State
            currentState.target = targetPose;
            currentState.events = &controlPoint->events;

            // Drive To Point
            directController.setTargetPose(*targetPose);
            directController.setReverse(controlPoint->isReversed);
            directController.update();
        }

        /**
         * Changes the path and resets the controller.
         * @param path The new path to follow.
         */
        void setPath(GeneratedPath *path)
        {
            currentPath = path;
            if (path != nullptr)
            {
                controlPoints = &currentPath->controlPoints;
                pathPoints = &currentPath->pathPoints;
            }
            reset();
        }

    private:
        static constexpr double ROTATIONAL_THRESHOLD = M_PI / 16; // rads
        static constexpr double LOOKAHEAD_DISTANCE = 8.0;         // in

        // Object Handles
        BaseChassis &chassis;
        OdomSource &odometry;

        // Shorthands
        GeneratedPath *currentPath;
        ControlPoints *controlPoints;
        PoseSequence *pathPoints;

        // Controller State
        DirectController directController;
        int robotPointIndex = 0;      // Closest path index to the robot
        int lookaheadPointIndex = 0;  // Closest path index to the lookahead
        int controlPointIndex = 0;    // Current control index of the event
        bool skipCheckpoints = false; // Whether the controller can skip checkpoints
    };
}