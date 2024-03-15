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
     * Runs though a list of controllers.
     */
    class ControllerList : public AutoController
    {
    public:
        /**
         * Constructs a new ControllerList.
         * @param controllers The controllers to run.
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
            while (controllerIndex < controllers.size())
            {
                controllers[controllerIndex]->runSync();
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
            if (controllerIndex >= controllers.size() || controllerIndex < 0)
                return nullptr;
            return controllers[controllerIndex];
        }

    private:
        std::vector<AutoController *> controllers;
        bool loop = false;
        int controllerIndex = 0;
    };
}