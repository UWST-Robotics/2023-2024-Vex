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
            : chassis(chassis), generatedPath(generatedPath), odometry(odometry)
        {
        }

        /**
         * Gets list of current events.
         * @return List of current events.
         */
        PathPoint *getControlPoint() override
        {
            auto &controlPoints = generatedPath.controlPoints;
            if (controlPointIndex >= controlPoints.size() || controlPointIndex < 0)
                return nullptr;
            return &controlPoints[controlPointIndex];
        }

        /**
         * Returns the current target point of the motion profile.
         * Controller tries to choose LOOKAHEAD_DISTANCE inches ahead of the robot.
         * @return The current target point of the motion profile.
         */
        Pose *getTargetPose() override
        {
            if (!generatedPath.isGenerated())
                return nullptr;

            auto &pathPoints = generatedPath.pathPoints;
            return &pathPoints[pathPointIndex];
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void _run() override
        {
            // Get Current Pose
            auto &controlPoints = generatedPath.controlPoints;
            auto &pathPoints = generatedPath.pathPoints;
            auto &lastPoint = pathPoints.back();

            while (!hasFinished)
            {
                // Check if paused
                if (isPaused)
                {
                    chassis.move(0, 0);
                    pros::delay(20);
                    continue;
                }

                // Get Current Pose
                auto currentPose = odometry.getPose();

                // Update Path Point Index
                double closestDistance = 100000;
                for (int i = pathPointIndex; i < pathPoints.size(); i++)
                {
                    auto point = pathPoints[i];
                    double distance = abs(LOOKAHEAD_DISTANCE - sqrt(pow(point.x - currentPose.x, 2) + pow(point.y - currentPose.y, 2)));
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        pathPointIndex = i;
                    }
                    if (i - pathPointIndex > LOOKAHEAD_MAX_INDICES)
                        break;
                }

                // Update Control Point Index
                if (controlPointIndex < controlPoints.size() - 1)
                {
                    auto controlPoint = controlPoints[controlPointIndex + 1];
                    double distanceSq = pow(controlPoint.x - currentPose.x, 2) + pow(controlPoint.y - currentPose.y, 2);
                    if (distanceSq < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE)
                        controlPointIndex++;
                }

                // Get Current Point
                auto targetPose = getTargetPose();
                auto controlPoint = getControlPoint();

                // Drive To Point
                driveTowards(*targetPose, controlPoint->isReversed);

                // Handle Finish
                bool withinRangeOfLastPoint = pow(lastPoint.x - currentPose.x, 2) + pow(lastPoint.y - currentPose.y, 2) < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE;
                bool lookingAtLastPoint = pathPointIndex == pathPoints.size() - 1;
                if (withinRangeOfLastPoint && lookingAtLastPoint)
                    hasFinished = true;

                pros::delay(20);
            }

            // Stop
            chassis.stop();
        }

        /**
         * Returns whether the controller has finished.
         * @return Whether the controller has finished.
         */
        bool isFinished() override
        {
            return hasFinished;
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void reset()
        {
            pathPointIndex = 0;
            auto startingPose = generatedPath.getStartingPose();
            if (startingPose != nullptr)
                odometry.setPose(*startingPose);
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
            double deltaX = (targetPose.x - currentPose.x) / LOOKAHEAD_DISTANCE;
            double deltaY = (targetPose.y - currentPose.y) / LOOKAHEAD_DISTANCE;
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
                forward = Curve::clamp(-1.0, 0.0, forward) * speed;
            else
                forward = Curve::clamp(0.0, 1.0, forward) * speed;
            turn = Curve::clamp(-1.0, 1.0, turn) * speed * normal;

            // Drive
            chassis.move(forward, turn);
        }

    private:
        static constexpr int LOOKAHEAD_MAX_INDICES = 15;
        static constexpr double LOOKAHEAD_DISTANCE = 6.0; // in
        static constexpr double EVENT_TRIGGER_RANGE = 6;  // in

        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation

        BaseChassis &chassis;
        GeneratedPath &generatedPath;
        OdomSource &odometry;
        int pathPointIndex = 0;    // Closest index of the lookahead point
        int controlPointIndex = 0; // Current index of the event

        bool isPaused = false;
        bool hasFinished = false;
    };
}