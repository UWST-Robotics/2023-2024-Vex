#pragma once
#include "../devils.h"

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(pathFile, "paths/testpath.txt");

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
              pathFile(PathFileReader::deserialize(g_pathFileData)),
              generatedPath(PathGenerator::generateLinear(pathFile)),
              pursuitController(chassis, generatedPath, odometry),
              collectionController(chassis, odometry, knownGameObjects),
              pathRenderer(&generatedPath),
              fieldRenderer(),
              gameObjectRenderer(&knownGameObjects),
              odomRenderer(&odometry),
              controlRenderer(&collectionController, &odometry),
              rectRenderer(&autonomousArea),
              statsRenderer(),
              displayStack({&pathRenderer,
                            &fieldRenderer,
                            &gameObjectRenderer,
                            &odomRenderer,
                            &controlRenderer,
                            &rectRenderer,
                            &statsRenderer})
        {
            // Auto Controllers
            collectionController.setInitController(&pursuitController);
            collectionController.setReturnController(&pursuitController);

            // Display Data
            statsRenderer.useOdomSource(&odometry);
            statsRenderer.useController(&controller);
            statsRenderer.useAutoController(&collectionController);
            gameObjectRenderer.useRect(&autonomousArea);
            displayStack.runAsync();
        }

        void opcontrol() override
        {
            chassis.setPose(*generatedPath.getStartingPose());

            // Set Seed
            srand(pros::millis());

            // Generate random test objects
            knownGameObjects.clear();
            for (int i = 0; i < 20; i++)
                knownGameObjects.push_back(fieldArea.getRandomPose());
            // gameObjectRenderer.useRect(&autonomousArea);

            EventTimer pauseTimer;
            auto pursuitTask = collectionController.runAsync();

            // Loop
            while (true)
            {
                // Handle Events
                auto currentEvents = collectionController.getCurrentEvents();
                if (currentEvents != nullptr)
                {
                    for (int i = 0; i < currentEvents->size(); i++)
                    {
                        auto currentEvent = currentEvents->at(i);
                        // Logger::info("Event: " + currentEvent.name);

                        // if (currentEvent.name == "pause" || currentEvent.name == "wack")
                        //     pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                    }

                    // Check For Fast/Slow
                    bool isFast = false;
                    bool isSlow = false;
                    for (int i = 0; i < currentEvents->size(); i++)
                    {
                        auto currentEvent = currentEvents->at(i);
                        if (currentEvent.name == "fast")
                            isFast = true;
                        else if (currentEvent.name == "slow")
                            isSlow = true;
                    }

                    // Set Max Speed
                    if (isFast)
                        chassis.setSpeed(1.0);
                    else if (isSlow)
                        chassis.setSpeed(0.25);
                    else
                        chassis.setSpeed(0.5);
                }

                // Pause
                if (pauseTimer.getRunning())
                {
                    chassis.stop();
                    pursuitTask.suspend();
                }
                else
                {
                    pursuitTask.resume();
                }

                // Additional Stats
                std::stringstream stream;
                stream << "Pause Timer: " << DisplayUtils::colorizeValue(pauseTimer.getTimeRemaining() / 5000, std::to_string((int)pauseTimer.getTimeRemaining()) + "ms");
                stream << "Speed: " << DisplayUtils::colorizeValue(chassis.getSpeed(), std::to_string((int)(chassis.getSpeed() * 100)) + "%");
                std::string additionalText = stream.str();
                statsRenderer.setAdditionalText(additionalText);

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
        PathFile pathFile;
        GeneratedPath generatedPath;
        PursuitController pursuitController;
        CollectionController collectionController;
        std::vector<GameObject> knownGameObjects;

        // Teleop Controller
        pros::Controller controller;

        // Display
        PathRenderer pathRenderer;
        FieldRenderer fieldRenderer;
        GameObjectRenderer gameObjectRenderer;
        OdomRenderer odomRenderer;
        ControlRenderer controlRenderer;
        RectRenderer rectRenderer;
        StatsRenderer statsRenderer;
        Display displayStack;
    };
}