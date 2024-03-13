#pragma once
#include "gameObject.hpp"
#include <vector>
#include "../utils/runnable.hpp"
#include "../odom/odomSource.hpp"

namespace devils
{
    /**
     * Manages the lifecycle of each game object
     */
    class GameObjectManager : public Runnable
    {
    public:
        void update() override
        {
        }

        void reset()
        {
            gameObjects.clear();
        }

        /**
         * Marks a game object as collected and removes it from the manager
         * @param object The game object to remove
         */
        void remove(GameObject &object)
        {
            gameObjects.erase(std::remove(gameObjects.begin(), gameObjects.end(), object), gameObjects.end());
        }

        /**
         * Adds a game object to the manager
         * @param object The game object to add
         */
        void add(GameObject object)
        {
            // Group game objects that are close together
            for (auto &other : gameObjects)
            {
                if (object.distanceTo(other) < GROUP_RADIUS)
                {
                    object.x = (object.x + other.x) / 2;
                    object.y = (object.y + other.y) / 2;
                    remove(other);
                }
            }

            // Add the object
            gameObjects.push_back(object);
        }

        /**
         * Gets all the active game objects
         * @return A vector of game objects
         */
        std::vector<GameObject> *getGameObjects()
        {
            return &gameObjects;
        }

    private:
        static constexpr double GROUP_RADIUS = 10.0;     // in
        static constexpr double COLLECTION_RADIUS = 3.0; // in

        std::vector<GameObject> gameObjects = {};
    };
}