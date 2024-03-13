#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/curve.hpp"
#include "../utils/pid.hpp"
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
         * @param generatedPath The generated path to follow.
         * @param odometry The odometry source to use.
         */
        PursuitController(BaseChassis &chassis, GeneratedPath &generatedPath, OdomSource &odometry)
            : chassis(chassis),
              generatedPath(generatedPath),
              odometry(odometry),
              controlPoints(generatedPath.controlPoints),
              pathPoints(generatedPath.pathPoints),
              lastPoint(generatedPath.pathPoints.back())
        {
        }

        std::vector<PathEvent> *getCurrentEvents() override
        {
            if (controlPointIndex >= controlPoints.size() || controlPointIndex < 0)
                return nullptr;
            return &controlPoints[controlPointIndex].events;
        }

        Pose *getTargetPose() override
        {
            if (lookaheadPointIndex >= pathPoints.size() || lookaheadPointIndex < 0)
                return nullptr;
            return &pathPoints[lookaheadPointIndex];
        }

        void update() override
        {
            // Get Current Pose
            auto currentPose = odometry.getPose();

            // Update Control Point Index
            if (controlPointIndex < controlPoints.size() - 1)
            {
                auto controlPoint = controlPoints[controlPointIndex + 1];
                double distance = controlPoint.distanceTo(currentPose);
                if (distance < LOOKAHEAD_DISTANCE)
                    controlPointIndex++;
            }
            int maxPathPointIndex = (this->controlPointIndex + 1) / generatedPath.dt;

            // Update Path Point Index
            double closestDistance = INT_MAX;
            for (int i = robotPointIndex; i < pathPoints.size(); i++)
            {
                auto point = pathPoints[i];
                double distance = point.distanceTo(currentPose);

                // Check if closest
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    robotPointIndex = i;
                }

                if (i > maxPathPointIndex)
                    break;
            }

            // Update Lookahead Point Index
            for (int i = lookaheadPointIndex; i < pathPoints.size(); i++)
            {
                auto targetPose = getTargetPose();
                if (targetPose->distanceTo(currentPose) > LOOKAHEAD_DISTANCE)
                    break;
                lookaheadPointIndex = i;
            }

            // Get Current Point
            auto targetPose = getTargetPose();
            bool isReversed = controlPoints[controlPointIndex].isReversed;

            // Handle Finish
            bool withinRangeOfLastPoint = currentPose.distanceTo(lastPoint) < LOOKAHEAD_DISTANCE;
            bool lookingAtLastPoint = robotPointIndex == pathPoints.size() - 1;
            if (withinRangeOfLastPoint && lookingAtLastPoint)
            {
                isFinished = true;
                chassis.stop();
            }

            // Drive To Point
            else
            {
                driveTowards(*targetPose, isReversed);
            }
        }

        bool getFinished() override
        {
            return isFinished;
        }

        void reset() override
        {
            robotPointIndex = 0;
            lookaheadPointIndex = 0;
            controlPointIndex = 0;
            isFinished = false;
            // auto startingPose = generatedPath.getStartingPose();
            // if (startingPose != nullptr)
            //     odometry.setPose(*startingPose);
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
                forward = Curve::clamp(-1.0, 0.0, forward);
            else
                forward = Curve::clamp(0.0, 1.0, forward);
            turn = Curve::clamp(-1.0, 1.0, turn) * normal;

            // Drive
            chassis.move(forward, turn);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 6.0; // in

        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation

        // Object Handles
        BaseChassis &chassis;
        GeneratedPath &generatedPath;
        OdomSource &odometry;

        // Shorthands
        std::vector<PathPoint> &controlPoints;
        std::vector<Pose> &pathPoints;
        Pose &lastPoint;

        // Controller State
        int robotPointIndex = 0;     // Closest path index to the robot
        int lookaheadPointIndex = 0; // Closest path index to the lookahead
        int controlPointIndex = 0;   // Current control index of the event
        bool isFinished = false;
    };
}