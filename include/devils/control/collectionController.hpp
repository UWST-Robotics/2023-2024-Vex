#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "autoController.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "pros/rtos.hpp"
#include "../utils/pid.hpp"
#include "../utils/curve.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../utils/polygon.hpp"

namespace devils
{
    /**
     * Controller for retriving game object
     */
    class CollectionController : public AutoController
    {
    public:
        /**
         * Constructs a new LinearController.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         * @param gameObjects The game objects to collect.
         * @param goalPose The pose to return to after collecting the game objects.
         */
        CollectionController(BaseChassis &chassis, OdomSource &odometry, GameObjectManager &gameObjectManager)
            : chassis(chassis),
              odometry(odometry),
              gameObjectManager(gameObjectManager)
        {
        }

        std::vector<PathEvent> *getCurrentEvents() override
        {
            return &NO_EVENTS;
        }

        Pose *getTargetPose() override
        {
            if (currentObject != nullptr)
                return currentObject;
            return nullptr;
        }

        void update() override
        {
            // Get the closest object
            auto currentPose = odometry.getPose();
            auto gameObjects = gameObjectManager.getGameObjects();

            double closestDistance = INT_MAX;
            currentObject = nullptr;
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
                    currentObject = &object;
                }
            }

            // Abort if no object is found
            if (currentObject == nullptr)
                return;

            // Check if object is in proximity
            bool hasObject = false;
            if (collectionSensor != nullptr)
                hasObject = collectionSensor->getProximity() < OPTICAL_PROXIMITY;
            else if (currentObject != nullptr)
            {
                auto distance = currentObject->distanceTo(currentPose);
                if (distance < COLLECTION_DISTANCE)
                    hasObject = true;
            }

            // If object is in proximity, return to the path
            if (hasObject)
            {
                gameObjectManager.remove(*currentObject);
                isFinished = true;
                Logger::info("CollectionController: Object Collected");
            }

            // Move to the object
            double deltaX = currentObject->x - currentPose.x;
            double deltaY = currentObject->y - currentPose.y;
            double normal = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;
            double forward = translationPID.update(deltaForward);
            double turn = rotationPID.update(deltaRotation);

            // Clamp Values
            forward = Curve::clamp(0.0, 1.0, forward);
            turn = Curve::clamp(-1.0, 1.0, turn) * normal;

            // Drive
            chassis.move(forward, turn);
        }

        void reset() override
        {
            isFinished = false;
            currentObject = nullptr;
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

    private:
        static constexpr double COLLECTION_DISTANCE = 4.0; // in
        static constexpr double OPTICAL_PROXIMITY = 0.5;   // %
        std::vector<PathEvent> NO_EVENTS = {};

        // PID
        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation

        // Required Components
        BaseChassis &chassis;
        OdomSource &odometry;
        GameObjectManager &gameObjectManager;

        // State
        bool isFinished = false;
        GameObject *currentObject = nullptr;

        // Optional Components
        AutoController *initializeController = nullptr;
        AutoController *returnController = nullptr;
        OpticalSensor *collectionSensor = nullptr;
        Polygon *collectionArea = nullptr;
    };
}