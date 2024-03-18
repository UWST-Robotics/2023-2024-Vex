#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "autoController.hpp"
#include "directController.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "pros/rtos.hpp"
#include "../utils/pid.hpp"
#include "../utils/curve.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../utils/polygon.hpp"
#include "../path/occupancyGrid.hpp"
#include "../display/pathRenderer.hpp"
#include "../path/pathFinder.hpp"

namespace devils
{
    /**
     * Controller for retriving game object
     */
    class CollectionController : public AutoController
    {
    public:
        /**
         * Constructs a new collection controller.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         * @param gameObjectManager The game objects to collect.
         * @param occupancyGrid The occupancy grid to use.
         */
        CollectionController(BaseChassis &chassis, OdomSource &odometry, GameObjectManager &gameObjectManager, OccupancyGrid &occupancyGrid)
            : chassis(chassis),
              odometry(odometry),
              gameObjectManager(gameObjectManager),
              pursuitController(chassis, odometry, nullptr, true),
              directController(chassis, odometry),
              occupancyGrid(occupancyGrid),
              targetLocation(odometry.getPose())
        {
        }

        std::vector<PathEvent> *getCurrentEvents() override
        {
            return &DEFAULT_EVENTS;
        }

        Pose *getTargetPose() override
        {
            return targetObject;
        }

        void update() override
        {
            // Get Current State
            auto currentPose = odometry.getPose();
            auto gameObjects = gameObjectManager.getGameObjects();

            // Get the closest object
            auto closestObject = _getClosestObject();

            // Abort if no objects on field
            if (closestObject == nullptr)
            {
                chassis.stop();
                return;
            }

            // Compare to the current pose
            // Recalculate path if the object location changes
            if (targetObject == nullptr || SWITCH_OBJECT_DISTANCE < closestObject->distanceTo(targetLocation))
            {
                // Set Target
                targetObject = closestObject;
                targetLocation = *targetObject;

                // Regenerate the path
                currentPath = PathFinder::generatePath(currentPose, *targetObject, occupancyGrid);
                pursuitController.setPath(&currentPath);
                if (pathRenderer != nullptr)
                    pathRenderer->setPath(currentPath);
            }

            // Check if object is picked up
            bool hasObject = false;
            auto objectDistance = targetObject->distanceTo(currentPose);
            if (collectionSensor != nullptr)
                hasObject = collectionSensor->getProximity() < OPTICAL_PROXIMITY;
            else
            {
                if (objectDistance < COLLECTION_DISTANCE)
                    hasObject = true;
            }

            // If object is in proximity, return to the path
            if (hasObject)
            {
                gameObjectManager.remove(*targetObject);
                isFinished = true;
                chassis.stop();
                Logger::info("CollectionController: Object Collected");
                return;
            }

            // If object is in proximity, chase it
            if (objectDistance < CHASE_DISTANCE || pursuitController.getFinished())
            {
                directController.setTargetPose(*targetObject);
                directController.update();
            }
            else
            {
                pursuitController.update();
            }
        }

        void reset() override
        {
            isFinished = false;
            pursuitController.reset();
        }

        bool getFinished() override
        {
            return isFinished;
        }

        /**
         * Uses an optical sensor to detect game objects in the robot's collection.
         * @param sensor The optical sensor to use.
         */
        void useCollectionSensor(OpticalSensor *sensor)
        {
            collectionSensor = sensor;
        }

        /**
         * Uses a polygon to define the area where game objects can be collected.
         * @param polygon The area to use.
         */
        void useCollectionArea(Polygon *polygon)
        {
            collectionArea = polygon;
        }

        /**
         * Uses a path renderer to visualize the path being followed.
         * @param renderer The path renderer to use.
         */
        void usePathRenderer(PathRenderer *renderer)
        {
            pathRenderer = renderer;
        }

        /**
         * Gets the closest object to the robot.
         */
        GameObject *_getClosestObject()
        {
            // Get Current State
            auto currentPose = odometry.getPose();
            auto gameObjects = gameObjectManager.getGameObjects();

            // Get the closest object
            double closestDistance = INT_MAX;
            GameObject *closestObject = nullptr;
            for (auto &object : *gameObjects)
            {
                // Calculate Distance
                auto distance = object.distanceTo(currentPose);
                if (distance < closestDistance)
                {
                    // Skip if object is not in the collection area
                    if (collectionArea != nullptr && !collectionArea->contains(object))
                        continue;

                    // Mark the object as the closest
                    closestDistance = distance;
                    closestObject = &object;
                }
            }

            return closestObject;
        }

    private:
        static constexpr double SWITCH_OBJECT_DISTANCE = 12.0; // in
        static constexpr double CHASE_DISTANCE = 12.0;         // in
        static constexpr double COLLECTION_DISTANCE = 4.0;     // in
        static constexpr double OPTICAL_PROXIMITY = 0.5;       // %
        std::vector<PathEvent> DEFAULT_EVENTS = {PathEvent("collection", "")};

        // PID
        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation

        // Required Components
        BaseChassis &chassis;
        OdomSource &odometry;
        GameObjectManager &gameObjectManager;
        OccupancyGrid &occupancyGrid;

        // State
        GeneratedPath currentPath;
        PursuitController pursuitController;
        DirectController directController;
        Pose targetLocation;
        GameObject *targetObject = nullptr;
        bool isFinished = false;

        // Optional Components
        OpticalSensor *collectionSensor = nullptr;
        Polygon *collectionArea = nullptr;
        PathRenderer *pathRenderer = nullptr;
    };
}