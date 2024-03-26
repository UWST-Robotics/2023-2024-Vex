#pragma once
#include "pros/rtos.hpp"
#include "../path/generatedPath.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "../chassis/chassis.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../geometry/polygon.hpp"
#include "../utils/pid.hpp"
#include "autoController.hpp"

namespace devils
{
    /**
     * Runs though a list of controllers in sequential order
     */
    class ControllerList : public AutoController
    {
    public:
        /**
         * Constructs a new ControllerList.
         * @param controllers The controllers to run.
         * @param loop Whether to loop the controllers.
         * @param timeout Maximum time to run the controllers before marking as finished. Measured in milliseconds.
         */
        ControllerList(std::initializer_list<AutoController *> controllers, bool loop = false, int timeout = -1)
            : controllers(controllers),
              loop(loop),
              timeout(timeout)
        {
        }

        void update() override
        {
            if (startTime < 0)
                startTime = pros::millis();
            auto controller = getCurrentController();
            if (controller != nullptr)
            {
                controller->update();
                if (controller->getFinished())
                    skip();
            }
        }

        void runSync() override
        {
            reset();
            while (!getFinished())
            {
                getCurrentController()->reset();
                getCurrentController()->runSync();
                controllerIndex++;
                pros::delay(20);
            }
        }

        Pose *getTargetPose() override
        {
            auto controller = getCurrentController();
            if (controller != nullptr)
                return controller->getTargetPose();
            return nullptr;
        }

        std::vector<PathEvent> &getCurrentEvents() override
        {
            auto controller = getCurrentController();
            if (controller != nullptr)
                return controller->getCurrentEvents();
            return NO_EVENTS;
        }

        void reset() override
        {
            startTime = pros::millis();
            controllerIndex = 0;
            for (auto controller : controllers)
                controller->reset();
        }

        bool getFinished() override
        {
            if (timeout > 0 && pros::millis() - startTime > timeout)
                return true;
            if (loop)
                return false;
            return controllerIndex >= controllers.size();
        }

        /**
         * Gets the current working controller.
         * @param searchChildren Recursively runs `getCurrentController` on each `ControllerList`.
         * @return The current controller
         */
        AutoController *getCurrentController(bool searchChildren = false)
        {
            AutoController *currentController = controllers[controllerIndex % controllers.size()];

            // Abort if not searching children
            if (!searchChildren)
                return currentController;

            // Loop through controller lists
            // Exits if it ever finds this controller
            ControllerList *list = nullptr;
            do
            {
                list = dynamic_cast<ControllerList *>(currentController);
                if (list != nullptr)
                    currentController = list->getCurrentController();
            } while (list != nullptr);

            return currentController;
        }

        /**
         * Skips the current controller and moves to the next one.
         */
        void skip()
        {
            // Increment the controller index
            controllerIndex++;

            // Reset the next controller
            auto currentController = getCurrentController();
            if (currentController != nullptr)
                currentController->reset();
        }

    private:
        std::vector<AutoController *> controllers;
        bool loop = false;
        int controllerIndex = 0;
        int timeout = -1;
        int startTime = -1;
    };
}