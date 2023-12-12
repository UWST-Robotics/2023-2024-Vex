#pragma once
#include "pathFileReader.hpp"
#include "pathFile.hpp"
#include "motionProfile.hpp"
#include "okapi/squiggles/squiggles.hpp"
#include "../utils/units.hpp"
#include "../utils/logger.hpp"
#include <vector>

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Generates a motion profile using the Squiggles library.
     */
    class SquigglesGenerator
    {
    public:
        /**
         * Creates a new motion profile.
         * @param maxVelocity The maximum velocity of the robot in inches per second.
         * @param maxAcceleration The maximum acceleration of the robot in inches per second squared.
         * @param maxJerk The maximum jerk of the robot in inches per second cubed.
         * @param robotTrackWidth The width of the robot in inches.
         */
        SquigglesGenerator(
            const float maxVelocity,
            const float maxAcceleration,
            const float maxJerk,
            const float robotTrackWidth)
            : constraints(Units::inToMeters(maxVelocity), Units::inToMeters(maxAcceleration), Units::inToMeters(maxJerk)),
              model(std::make_shared<squiggles::TankModel>(Units::inToMeters(robotTrackWidth), constraints)),
              generator(constraints, model, DT)
        {
        }

        /**
         * Generates the motion profile from the path file and stores it in motionProfile.
         * This is computationally expensive, so it should only be done once before the match.
         * @param motionProfile The motion profile to store the generated profile in.
         */
        void generate(MotionProfile *motionProfile)
        {
            auto devilsControl = devils::PathFileReader::readFromSD();
            auto squigglesControl = _devilsToSquigglesPose(devilsControl.points);
            auto squigglesProfile = generator.generate(squigglesControl);
            auto devilsProfile = _squigglesToDevilsProfile(squigglesProfile);
            motionProfile->setProfilePoints(devilsControl.points, devilsProfile, DT);
        }

        /**
         * Converts a list of squiggles::ProfilePoint to a list of devils::Pose.
         * @param squigglesPoints The list of squiggles::ProfilePoint to convert.
         * @return The list of devils::Pose.
         */
        const std::vector<ProfilePose> _squigglesToDevilsProfile(const std::vector<squiggles::ProfilePoint> &squigglesPoints)
        {
            std::vector<ProfilePose> devilsPoints;
            for (auto squigglesPoint : squigglesPoints)
            {
                devilsPoints.push_back({
                    Units::metersToIn(squigglesPoint.vector.pose.x),
                    Units::metersToIn(squigglesPoint.vector.pose.y),
                    Units::radToDeg(squigglesPoint.vector.pose.yaw),
                    Units::metersToIn(squigglesPoint.wheel_velocities[0]),
                    Units::metersToIn(squigglesPoint.wheel_velocities[1]),
                });
            }
            return devilsPoints;
        }

        /**
         * Converts a list of devils::Pose to a list of squiggles::Pose.
         * @param devilPoints The list of devils::Pose to convert.
         * @return The list of squiggles::Pose.
         */
        const std::vector<squiggles::Pose> _devilsToSquigglesPose(const std::vector<PathPoint> &devilPoints)
        {
            std::vector<squiggles::Pose> squigglesPoints;
            for (auto devilPoint : devilPoints)
            {
                squigglesPoints.push_back({
                    Units::inToMeters(devilPoint.x),
                    Units::inToMeters(devilPoint.y),
                    Units::degToRad(devilPoint.rotation),
                });
            }
            return squigglesPoints;
        }

    private:
        static constexpr double DT = 0.1; // seconds

        squiggles::Constraints constraints;
        std::shared_ptr<squiggles::TankModel> model;
        squiggles::SplineGenerator generator;
    };
}