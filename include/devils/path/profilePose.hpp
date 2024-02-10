#pragma once

namespace devils
{
    struct ProfilePose
    {
        /**
         * The x position of the robot in inches.
         */
        double x = 0;

        /**
         * The y position of the robot in inches.
         */
        double y = 0;

        /**
         * The rotation of the robot in radians.
         */
        double rotation = 0;

        /**
         * The velocity of the left wheel in inches per second.
         */
        double leftWheelVelocity = -1;

        /**
         * The velocity of the right wheel in inches per second.
         */
        double rightWheelVelocity = -1;

        /**
         * Whether or not the point is going in the reverse direction.
         */
        bool isReversed = false;
    };
}