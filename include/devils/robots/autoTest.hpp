#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents an autonomous test robot
     */
    struct AutoTest : public Robot
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        AutoTest()
            : master(pros::E_CONTROLLER_MASTER),
              motionRenderer(&motionProfile),
              odomRenderer(&dummyOdometry),
              teleopDisplay({&motionRenderer, &fieldRenderer, &odomRenderer, &statsRenderer})
        {
            // Motion Profile
            auto generator = LinearGenerator();
            generator.generate(&motionProfile);

            // Add Stats
            statsRenderer.useOdomSource(&dummyOdometry);
            statsRenderer.useController(&master);
        }

        void disabled()
        {
            // Loop
            while (true)
            {
                // Display
                teleopDisplay.update();

                // Delay
                pros::delay(20);
            }
        }

        void autonomous()
        {
            // Loop
            while (true)
            {
                // Display
                teleopDisplay.update();

                // Delay
                pros::delay(20);
            }
        }

        void opcontrol()
        {
            // Loop
            while (true)
            {
                // Display
                teleopDisplay.update();

                // Random Odometry
                auto pose = dummyOdometry.getPose();
                pose.x = cos(pose.rotation) * 30;
                pose.y = sin(pose.rotation) * 30;
                pose.rotation += M_PI * 0.01;
                dummyOdometry.setPose(pose);

                // Delay
                pros::delay(20);
            }
        }

    private:
        // Subsystems
        MotionProfile motionProfile;
        DummyOdometry dummyOdometry;

        // Teleop Controller
        pros::Controller master;

        // Display
        MotionRenderer motionRenderer;
        FieldRenderer fieldRenderer;
        OdomRenderer odomRenderer;
        StatsRenderer statsRenderer;
        Display teleopDisplay;
    };
}