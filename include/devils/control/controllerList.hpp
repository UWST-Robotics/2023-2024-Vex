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
         * @param loop Whether to loop the controllers. `isFinished` will always return false if this is true.
         */
        ControllerList(std::initializer_list<AutoController *> controllers, bool loop = false)
            : controllers(controllers),
              loop(loop)
        {
        }

        void update() override
        {
            auto controller = getCurrentController();
            if (controller != nullptr)
                controller->update();
        }

        void runSync() override
        {
            while (!getFinished())
            {
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

        std::vector<PathEvent> *getCurrentEvents() override
        {
            auto controller = getCurrentController();
            if (controller != nullptr)
                return controller->getCurrentEvents();
            return nullptr;
        }

        void reset() override
        {
            controllerIndex = 0;
            for (auto controller : controllers)
                controller->reset();
        }

        bool getFinished() override
        {
            if (loop)
                return false;
            return controllerIndex >= controllers.size();
        }

        AutoController *getCurrentController()
        {
            return controllers[controllerIndex % controllers.size()];
        }

    private:
        std::vector<AutoController *> controllers;
        bool loop = false;
        int controllerIndex = 0;
    };
}