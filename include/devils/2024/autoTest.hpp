#pragma once
#include "../devils.h"
#include <time.h>

#define __ARM_NEON
#include "incbin/incbin.h"

#define INCBIN_PREFIX g_
INCTXT(startPath, "paths/testpath.txt");
INCTXT(scorePath, "paths/score.txt");
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

              startPathFile(PathFileReader::deserialize(g_startPathData)),
              scorePathFile(PathFileReader::deserialize(g_scorePathData)),
              occupancyGrid(OccupancyFileReader::deserialize(g_occupancyGridData)),

              startPath(PathGenerator::generateLinear(startPathFile)),
              scorePath(PathGenerator::generateLinear(scorePathFile)),
              startPursuitController(chassis, startPath, odometry),
              scorePursuitController(chassis, scorePath, odometry),
              collectionController(chassis, odometry, gameObjectManager),
              subController({&collectionController, &scorePursuitController}, true),
              mainController({&startPursuitController, &subController}),
              pathRendererA(&startPath),
              pathRendererB(&scorePath),
              fieldRenderer(),
              gameObjectRenderer(gameObjectManager),
              odomRenderer(&odometry),
              controlRenderer(&mainController, &odometry),
              polygonRenderer(&autonomousPolygon),
              pathPickerRenderer(),
              occupancyRenderer(occupancyGrid),
              statsRenderer(),
              displayStack({&pathRendererA,
                            &pathRendererB,
                            &fieldRenderer,
                            &gameObjectRenderer,
                            &odomRenderer,
                            &controlRenderer,
                            &polygonRenderer,
                            &occupancyRenderer,
                            //&pathPickerRenderer,
                            &statsRenderer})
        {
            // Auto Controllers
            collectionController.useCollectionArea(&autonomousPolygon);

            // Display Data
            statsRenderer.useOdomSource(&odometry);
            statsRenderer.useController(&controller);
            statsRenderer.useAutoController(&collectionController);
            gameObjectRenderer.useArea(&autonomousPolygon);
            displayStack.runAsync();
        }

        void opcontrol() override
        {
            chassis.setPose(*startPath.getStartingPose());

            // Set Seed
            srand(10);

            // Generate random test objects
            gameObjectManager.reset();

            EventTimer pauseTimer;
            auto autoTask = mainController.runAsync();

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
                if (pros::millis() % 3000 <= 20)
                {
                    auto gameObject = GameObject(autonomousPolygon.getRandomPose());
                    gameObjectManager.add(gameObject);
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
        PathFile startPathFile;
        PathFile scorePathFile;
        OccupancyGrid occupancyGrid;

        // Auto Controller Stack
        PursuitController startPursuitController;
        PursuitController scorePursuitController;
        CollectionController collectionController;
        ControllerList mainController;
        ControllerList subController;

        // Autonomous
        GeneratedPath startPath;
        GeneratedPath scorePath;
        GameObjectManager gameObjectManager;

        // Teleop Controller
        pros::Controller controller;

        // Display
        PathRenderer pathRendererA;
        PathRenderer pathRendererB;
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