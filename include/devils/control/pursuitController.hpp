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
            auto &pathPoints = generatedPath.pathPoints;
            if (lookaheadPointIndex >= pathPoints.size() || lookaheadPointIndex < 0)
                return nullptr;
            return &pathPoints[lookaheadPointIndex];
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
                auto controlPoint = getControlPoint();

                // Drive To Point
                driveTowards(*targetPose, controlPoint->isReversed);

                // Handle Finish
                bool withinRangeOfLastPoint = currentPose.distanceTo(lastPoint) < LOOKAHEAD_DISTANCE;
                bool lookingAtLastPoint = robotPointIndex == pathPoints.size() - 1;
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
            robotPointIndex = 0;
            lookaheadPointIndex = 0;
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
                forward = Curve::clamp(-1.0, 0.0, forward) * speed;
            else
                forward = Curve::clamp(0.0, 1.0, forward) * speed;
            turn = Curve::clamp(-1.0, 1.0, turn) * speed * normal;

            // Drive
            chassis.move(forward, turn);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 6.0; // in

        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation

        BaseChassis &chassis;
        GeneratedPath &generatedPath;
        OdomSource &odometry;
        int robotPointIndex = 0;     // Closest path index to the robot
        int lookaheadPointIndex = 0; // Closest path index to the lookahead
        int controlPointIndex = 0;   // Current control index of the event

        bool isPaused = false;
        bool hasFinished = false;
    };
}