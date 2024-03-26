#pragma once
#include "autoController.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/polygon.hpp"
#include "../path/occupancyGrid.hpp"
#include "../path/pathFinder.hpp"
#include "../odom/odomSource.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "../control/findController.hpp"
#include "../control/directController.hpp"
#include "../display/pathRenderer.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../hardware/visionSensor.hpp"

namespace devils
{
    /**
     * Controller drives to the closest game object and collects it.
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
              findController(chassis, odometry, occupancyGrid),
              directController(chassis, odometry)
        {
        }

        std::vector<PathEvent> &getCurrentEvents() override
        {
            if (isChasing)
                return CHASE_EVENTS;
            else
                return COLLECTION_EVENTS;
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
            targetObject = _getClosestObject();

            // Abort if no objects on field
            if (targetObject == nullptr)
            {
                chassis.stop();
                return;
            }

            // Set the target object
            findController.setTargetPose(*targetObject);

            // Check if object is picked up
            bool hasObject = false;
            auto objectDistance = targetObject->distanceTo(currentPose);
            if (collectionSensor != nullptr)
                hasObject = collectionSensor->getProximity() > OPTICAL_PROXIMITY;
            else
                hasObject = objectDistance < COLLECTION_DISTANCE;

            // Mark object as collected
            if (hasObject)
            {
                isFinished = true;
                gameObjectManager.remove(*targetObject);
                chassis.stop();
                return;
            }

            // If object is in proximity, chase it
            isChasing = objectDistance < CHASE_DISTANCE || findController.getFinished();
            if (isChasing)
            {
                directController.setTargetPose(*targetObject);
                directController.update();
            }
            else
            {
                findController.update();
            }
        }

        void reset() override
        {
            isFinished = false;
            findController.reset();
            directController.reset();
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
         * Uses a vision sensor to chase game objects.
         * @param sensor The vision sensor to use.
         */
        void useVisionSensor(VisionSensor *sensor)
        {
            // TODO: Implement
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
        // Constants
        static constexpr double CHASE_DISTANCE = 18.0;     // in
        static constexpr double COLLECTION_DISTANCE = 4.0; // in
        static constexpr double OPTICAL_PROXIMITY = 0.5;   // %
        std::vector<PathEvent> COLLECTION_EVENTS = {PathEvent("collection", "")};
        std::vector<PathEvent> CHASE_EVENTS = {PathEvent("chase", "")};

        // Required Components
        BaseChassis &chassis;
        OdomSource &odometry;
        GameObjectManager &gameObjectManager;

        // State
        GeneratedPath currentPath;
        FindController findController;
        DirectController directController;
        GameObject *targetObject = nullptr;
        bool isFinished = false;
        bool isChasing = false;

        // Optional Components
        OpticalSensor *collectionSensor = nullptr;
        Polygon *collectionArea = nullptr;
        PathRenderer *pathRenderer = nullptr;
    };
}