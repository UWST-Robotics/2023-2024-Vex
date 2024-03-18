#pragma once
#include "../devils.h"
#include <time.h>

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(occupancyGrid, "paths/occupancy.txt");

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
              occupancyGrid(OccupancyFileReader::deserialize(g_occupancyGridData)),
              collectionController(chassis, odometry, gameObjectManager, occupancyGrid),
              controllerStack({&collectionController}, true),
              pathRenderer(nullptr),
              fieldRenderer(),
              gameObjectRenderer(gameObjectManager),
              odomRenderer(&odometry),
              controlRenderer(&controllerStack, &odometry),
              polygonRenderer(&autonomousPolygon),
              pathPickerRenderer(),
              occupancyRenderer(occupancyGrid),
              statsRenderer(),
              displayStack({&pathRenderer,
                            &fieldRenderer,
                            &gameObjectRenderer,
                            &odomRenderer,
                            &controlRenderer,
                            //&polygonRenderer,
                            &occupancyRenderer,
                            //&pathPickerRenderer,
                            &statsRenderer})
        {
            // Auto Controllers
            collectionController.usePathRenderer(&pathRenderer);

            // Display Data
            statsRenderer.useOdomSource(&odometry);
            statsRenderer.useController(&controller);
            statsRenderer.useAutoController(&collectionController);
            // gameObjectRenderer.useArea(&autonomousPolygon);
            displayStack.runAsync();
        }

        void opcontrol() override
        {
            // chassis.setPose(*startPath.getStartingPose());

            // Set Seed
            srand(10);

            // Generate random test objects
            gameObjectManager.reset();

            EventTimer pauseTimer;
            auto autoTask = controllerStack.runAsync();

            // Loop
            while (true)
            {
                // Handle Events
                auto currentEvents = controllerStack.getCurrentEvents();
                if (currentEvents != nullptr)
                {
                    for (int i = 0; i < currentEvents->size(); i++)
                    {
                        auto currentEvent = currentEvents->at(i);
                        if (currentEvent.name == "pause" || currentEvent.name == "wack")
                            pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
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
                    autoTask.suspend();
                }
                else
                {
                    autoTask.resume();
                }

                // Add Game Object
                /*
                if (pros::millis() % 3000 <= 20)
                {
                    auto gameObject = GameObject(fieldArea.getRandomPose());
                    gameObjectManager.add(gameObject);
                }*/

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
        Polygon fieldArea = Polygon({Pose(-72, -72),
                                     Pose(72, -72),
                                     Pose(72, 72),
                                     Pose(-72, 72)});
        Polygon autonomousPolygon = Polygon({Pose(6, 0),
                                             Pose(42, 0),
                                             Pose(42, 32),
                                             Pose(68, 32),
                                             Pose(68, 46),
                                             Pose(46, 68),
                                             Pose(6, 68)});

        // Chassis
        DummyChassis chassis;
        OdomSource &odometry = (OdomSource &)chassis;

        // Data Files
        OccupancyGrid occupancyGrid;

        // Auto Controller Stack
        CollectionController collectionController;
        ControllerList controllerStack;

        // Autonomous
        GeneratedPath startPath;
        GeneratedPath scorePath;
        GameObjectManager gameObjectManager;

        // Teleop Controller
        pros::Controller controller;

        // Display
        PathRenderer pathRenderer;
        FieldRenderer fieldRenderer;
        GameObjectRenderer gameObjectRenderer;
        OdomRenderer odomRenderer;
        ControlRenderer controlRenderer;
        PolygonRenderer polygonRenderer;
        PathPickerRenderer pathPickerRenderer;
        OccupancyRenderer occupancyRenderer;
        StatsRenderer statsRenderer;
        Display displayStack;
    };
}