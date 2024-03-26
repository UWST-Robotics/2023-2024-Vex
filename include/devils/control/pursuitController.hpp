#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/pid.hpp"
#include "../geometry/lerp.hpp"
#include "autoController.hpp"
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
              skipCheckpoints(skipCheckpoints),
              currentPath(path)
        {
            setPath(path);
        }

        void reset() override
        {
            AutoController::reset();
            robotPointIndex = 0;
            lookaheadPointIndex = 0;
            controlPointIndex = 0;
        }

        void update() override
        {
            // Abort if path is missing
            if (currentPath == nullptr || pathPoints == nullptr || controlPoints == nullptr)
                return;

            // Get Current Pose
            auto currentPose = odometry.getPose();

            // Update Control Point Index
            if (controlPointIndex < controlPoints->size() - 1)
            {
                auto &controlPoint = controlPoints->at(controlPointIndex + 1);
                double distance = controlPoint.distanceTo(currentPose);
                if (distance < LOOKAHEAD_DISTANCE)
                    controlPointIndex++;
            }
            PathPoint *controlPoint = &controlPoints->at(controlPointIndex);
            int checkpointPathIndex = (this->controlPointIndex + 1) / currentPath->dt;

            // Update Path Point Index
            double closestDistance = INT_MAX;
            for (int i = robotPointIndex; i < pathPoints->size(); i++)
            {
                auto point = pathPoints->at(i);
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

            // Update State
            currentState.target = targetPose;
            currentState.events = controlPoint->events;

            // Get Current Point
            bool isReversed = controlPoint->isReversed;

            // Handle Finish
            auto lastPoint = pathPoints->back();
            bool withinRangeOfLastPoint = currentPose.distanceTo(lastPoint) < LOOKAHEAD_DISTANCE;
            bool lookingAtLastPoint = robotPointIndex == pathPoints->size() - 1;
            if (withinRangeOfLastPoint && lookingAtLastPoint)
            {
                Logger::debug("Finished path");
                currentState.isFinished = true;
                chassis.stop();
            }
            // Drive To Point
            else
            {
                driveTowards(*targetPose, isReversed);
            }
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

        /**
         * Drives the robot towards the target point.
         * @param targetPose The target point to drive towards.
         * @param isReversed Whether the robot is driving in reverse.
         */
        void driveTowards(Pose &targetPose, bool isReversed = false)
        {
            // Get Current Pose
            auto currentPose = odometry.getPose();

            // Calculate Forward & Turn
            double deltaX = targetPose.x - currentPose.x;
            double deltaY = targetPose.y - currentPose.y;
            double normal = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

            // Handle Reversed
            if (isReversed)
                deltaRotation = Units::diffRad(deltaRotation, M_PI);

            // Calculate PID
            double forward = translationPID.update(deltaForward);
            double turn = rotationPID.update(deltaRotation);

            // Clamp Values
            if (isReversed)
                forward = std::clamp(forward, -1.0, 0.0);
            else
                forward = std::clamp(forward, 0.0, 1.0);
            turn = std::clamp(turn, -1.0, 1.0) * normal;

            // Drive
            chassis.move(forward, turn);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 6.0; // in

        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation

        // Object Handles
        BaseChassis &chassis;
        OdomSource &odometry;

        // Shorthands
        GeneratedPath *currentPath;
        std::vector<PathPoint> *controlPoints;
        std::vector<Pose> *pathPoints;

        // Controller State
        int robotPointIndex = 0;      // Closest path index to the robot
        int lookaheadPointIndex = 0;  // Closest path index to the lookahead
        int controlPointIndex = 0;    // Current control index of the event
        bool skipCheckpoints = false; // Whether the controller can skip checkpoints
    };
}