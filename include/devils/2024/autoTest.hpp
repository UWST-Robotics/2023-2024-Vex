#pragma once
#include "../devils.h"
#include "systems/pjAutoController.hpp"
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
         * Creates a new instance of Auto Tester.
         */
        AutoTest()
        {
            // Auto Controllers
            autoController.usePathRenderer(displayStack.getRenderer<PathRenderer>());
            autoController.useCollectionArea(&autonomousPolygon);

            // Stats Renderer
            statsRenderer->useOdomSource(&odometry);
            statsRenderer->useController(&mainController);
            statsRenderer->useAutoController(&autoController);

            // Game Object Renderer
            GameObjectRenderer *gameObjectRenderer = displayStack.getRenderer<GameObjectRenderer>();
            gameObjectRenderer->useArea(&autonomousPolygon);

            // Run Display
            displayStack.runAsync();
        }

        void opcontrol() override
        {
            // Set Starting Pose
            chassis.setPose(*autoController.getStartingPose());

            // Autonomous
            EventTimer pauseTimer;
            auto autoTask = autoController.runAsync();

            // Loop
            while (true)
            {
                // Handle Events
                auto &currentEvents = autoController.getCurrentEvents();
                chassis.setSpeed(0.7);
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents.at(i);
                    if (currentEvent.name == "pause" || currentEvent.name == "wack")
                        pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                    else if (currentEvent.name == "fast")
                        chassis.setSpeed(1.0);
                    else if (currentEvent.name == "slow")
                        chassis.setSpeed(0.25);
                }

                // Pause Controller
                if (pauseTimer.getRunning() && PAUSE_IN_AUTONOMOUS)
                {
                    chassis.stop();
                    autoTask.suspend();
                }
                else
                {
                    autoTask.resume();
                }

                // Append Additional Stats
                std::stringstream stream;
                stream << "Pause Timer: " << DisplayUtils::colorizeValue(pauseTimer.getTimeRemaining() / 5000, std::to_string((int)pauseTimer.getTimeRemaining()) + "ms");
                stream << "Speed: " << DisplayUtils::colorizeValue(chassis.getSpeed(), std::to_string((int)(chassis.getSpeed() * 100)) + "%");
                std::string additionalText = stream.str();
                statsRenderer->setAdditionalText(additionalText);

                // Delay
                pros::delay(20);
            }
        }

    private:
        static constexpr bool PAUSE_IN_AUTONOMOUS = false;
        static constexpr uint8_t VISION_SENSOR_PORT = 1;
        static constexpr double VISION_SENSOR_HEIGHT = 12; // inches
        static constexpr double VISION_SENSOR_PITCH = 45;  // degrees

        // Geometry
        Polygon fullField = Polygon({Pose(-72, -72),
                                     Pose(72, -72),
                                     Pose(72, 72),
                                     Pose(-72, 72)});
        Polygon collectionPolygon = Polygon({Pose(-72, -72),
                                             Pose(-72, -30),
                                             Pose(-40, -30),
                                             Pose(-40, 30),
                                             Pose(-72, 30),
                                             Pose(-72, 72),
                                             Pose(72, 72),
                                             Pose(72, 30),
                                             Pose(40, 30),
                                             Pose(40, -30),
                                             Pose(72, -30),
                                             Pose(72, -72),
                                             Pose(-72, -72)});
        Polygon autonomousPolygon = Polygon({Pose(6, 0),
                                             Pose(42, 0),
                                             Pose(42, 32),
                                             Pose(68, 32),
                                             Pose(68, 46),
                                             Pose(46, 68),
                                             Pose(6, 68)});
        Perspective visionSensorPerspective = PerspectiveFactory::buildLegacyVisionSensor(VISION_SENSOR_HEIGHT, VISION_SENSOR_PITCH);

        // Chassis
        DummyChassis chassis = DummyChassis();
        OdomSource &odometry = (OdomSource &)chassis;
        // VisionSensor visionSensor = VisionSensor("IntakeCamera", VISION_SENSOR_PORT);

        // Data Files
        OccupancyGrid occupancyGrid = OccupancyFileReader::deserialize(g_occupancyGridData);

        // Autonomous
        GameObjectManager gameObjectManager = GameObjectManager();

        // Auto Controller
        PJAutoController autoController = PJAutoController(chassis, odometry, gameObjectManager, occupancyGrid);

        // Display
        Display displayStack = Display({new PathRenderer(nullptr),
                                        new GridRenderer(),
                                        new FieldRenderer(),
                                        new GameObjectRenderer(gameObjectManager),
                                        new OdomRenderer(&odometry),
                                        new ControlRenderer(&autoController, &odometry),
                                        new PolygonRenderer(&autonomousPolygon),
                                        // new PathPickerRenderer(),
                                        //  new OccupancyRenderer(occupancyGrid),
                                        new StatsRenderer()});
        StatsRenderer *statsRenderer = displayStack.getRenderer<StatsRenderer>();
    };
}