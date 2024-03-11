#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents an autonomous test robot
     */
    struct AutoTest : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        AutoTest()
            : controller(pros::E_CONTROLLER_MASTER),
              linearGenerator(motionProfile),
              autoController(chassis, motionProfile, odometry),
              motionRenderer(&motionProfile),
              odomRenderer(&odometry),
              rectRenderer(&autonomousArea),
              gameObjectRenderer(&knownGameObjects),
              controlRenderer(&autoController),
              teleopDisplay({&motionRenderer, &fieldRenderer, &odomRenderer, &controlRenderer, &gameObjectRenderer, &rectRenderer, &statsRenderer})
        {
            // Motion Profile
            auto generator = LinearGenerator();
            generator.generate(&motionProfile);

            // Add Stats
            statsRenderer.useOdomSource(&odometry);
            statsRenderer.useController(&controller);
            statsRenderer.useAutoController(&autoController);
        }

        void disabled() override
        {
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            chassis.setPose(motionProfile.getStartingPose());

            // Generate random test objects
            knownGameObjects.clear();
            for (int i = 0; i < 20; i++)
                knownGameObjects.push_back(fieldArea.getRandomPose());
            gameObjectRenderer.useRect(&autonomousArea);

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
                stream << "Speed: " << DisplayUtils::colorizeValue(autoController.getSpeed(), std::to_string((int)(autoController.getSpeed() * 100)) + "%");
                std::string additionalText = stream.str();
                statsRenderer.setAdditionalText(additionalText);

                // Display
                teleopDisplay.update();

                // Delay
                pros::delay(20);
            }
        }

    private:
        // Field Area
        Rect fieldArea = Rect(-72, -72, 144, 144);
        Rect autonomousArea = Rect(-36, 0, 72, 48);

        // Chassis
        DummyChassis chassis;
        OdomSource &odometry = (OdomSource &)chassis;

        // Autonomous
        MotionProfile motionProfile;
        LinearGenerator linearGenerator;
        PursuitController autoController;
        std::vector<GameObject> knownGameObjects;

        // Teleop Controller
        pros::Controller controller;

        // Display
        MotionRenderer motionRenderer;
        FieldRenderer fieldRenderer;
        OdomRenderer odomRenderer;
        ControlRenderer controlRenderer;
        GameObjectRenderer gameObjectRenderer;
        RectRenderer rectRenderer;
        StatsRenderer statsRenderer;
        Display teleopDisplay;
    };
}