#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "autoController.hpp"
#include "../gameobject/gameobject.hpp"
#include "pros/rtos.hpp"
#include "../utils/pid.hpp"
#include "../utils/curve.hpp"
#include "../odom/odomSource.hpp"

namespace devils
{
    /**
     * Controller for grabbing a game object and returning to a path or position.
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
        CollectionController(BaseChassis &chassis, OdomSource &odometry, std::vector<GameObject> &gameObjects, Pose goalPose)
            : chassis(chassis),
              odometry(odometry),
              gameObjects(gameObjects),
              goalPose(goalPose)
        {
            // TODO: Implement
        }

    private:
        BaseChassis &chassis;
        OdomSource &odometry;
        std::vector<GameObject> &gameObjects;
        Pose &goalPose;
    };
}