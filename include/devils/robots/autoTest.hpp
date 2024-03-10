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
            : controller(pros::E_CONTROLLER_MASTER),
              linearGenerator(motionProfile),
              autoController(chassis, motionProfile, odometry),
              motionRenderer(&motionProfile),
              odomRenderer(&odometry),
              controlRenderer(&autoController),
              teleopDisplay({&motionRenderer, &fieldRenderer, &odomRenderer, &controlRenderer, &statsRenderer})
        {
            // Motion Profile
            auto generator = LinearGenerator();
            generator.generate(&motionProfile);

            // Add Stats
            statsRenderer.useOdomSource(&odometry);
            statsRenderer.useController(&controller);
            statsRenderer.useAutoController(&autoController);
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
        }

        void opcontrol()
        {

            chassis.setPose(motionProfile.getStartingPose());

            // Start Auto Task
            pros::Task autoTask = autoController.run();
            AutoTimer pauseTimer;

            // Loop
            while (true)
            {
                // Handle Events
                auto currentEvents = autoController.getCurrentEvents();
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents[i];
                    Logger::info("Event: " + currentEvent.name);

                    if (currentEvent.name == "pause" || currentEvent.name == "wack")
                        pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                }

                // Check For Fast/Slow
                bool isFast = false;
                bool isSlow = false;
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents[i];
                    if (currentEvent.name == "fast")
                        isFast = true;
                    else if (currentEvent.name == "slow")
                        isSlow = true;
                }

                // Set Max Speed
                if (isFast)
                    autoController.setSpeed(1.0);
                else if (isSlow)
                    autoController.setSpeed(0.25);
                else
                    autoController.setSpeed(0.5);

                // Pause
                if (pauseTimer.getRunning())
                    autoTask.suspend();
                else
                    autoTask.resume();

                // Additional Stats
                std::stringstream stream;
                stream << "Pause Timer: " << DisplayUtils::colorizeValue(pauseTimer.getTimeRemaining() / 5000, std::to_string((int)pauseTimer.getTimeRemaining()) + "ms");
                stream << "Fast: " << DisplayUtils::colorizeValue(isFast, isFast ? "Yes" : "No");
                stream << "Slow: " << DisplayUtils::colorizeValue(isSlow, isSlow ? "Yes" : "No");
                std::string additionalText = stream.str();
                statsRenderer.setAdditionalText(additionalText);

                // Display
                teleopDisplay.update();

                // Delay
                pros::delay(20);
            }
        }

    private:
        // Chassis
        DummyChassis chassis;
        OdomSource &odometry = (OdomSource &)chassis;

        // Autonomous
        MotionProfile motionProfile;
        LinearGenerator linearGenerator;
        PursuitController autoController;

        // Teleop Controller
        pros::Controller controller;

        // Display
        MotionRenderer motionRenderer;
        FieldRenderer fieldRenderer;
        OdomRenderer odomRenderer;
        ControlRenderer controlRenderer;
        StatsRenderer statsRenderer;
        Display teleopDisplay;
    };
}