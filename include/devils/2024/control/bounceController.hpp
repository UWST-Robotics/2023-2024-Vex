#pragma once
#include "pros/rtos.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Runs the chassis in a forward and reverse motion.
     */
    class BounceController : public ControllerList
    {
    public:
        BounceController(BaseChassis &chassis)
            : chassis(chassis),
              forwardController(chassis, forwardDuration, forwardSpeed),
              reverseController(chassis, reverseDuration, reverseSpeed),
              ControllerList({&reverseController, &forwardController}, true)
        {
        }
        BounceController(BaseChassis &chassis, double forwardSpeed, double reverseSpeed)
            : chassis(chassis),
              forwardController(chassis, forwardDuration, forwardSpeed),
              reverseController(chassis, reverseDuration, reverseSpeed),
              ControllerList({&reverseController, &forwardController}, true)
        {
        }
        BounceController(BaseChassis &chassis, double forwardSpeed, double reverseSpeed, int forwardDuration, int reverseDuration)
            : chassis(chassis),
              forwardController(chassis, forwardDuration, forwardSpeed),
              reverseController(chassis, reverseDuration, reverseSpeed),
              ControllerList({&reverseController, &forwardController}, true)
        {
        }

        /**
         * Sets the rotation of the chassis.
         * @param rotationalSpeed The rotational speed to set.
         */
        void setRotation(double rotationalSpeed)
        {
            forwardController.setSpeeds(forwardSpeed, rotationalSpeed);
            reverseController.setSpeeds(reverseSpeed, rotationalSpeed);
        }

        /**
         * Sets the speeds for the forward and reverse motions.
         * @param forwardSpeed The forward speed to set.
         * @param reverseSpeed The reverse speed to set.
         */
        void setSpeeds(double forwardSpeed, double reverseSpeed)
        {
            this->forwardSpeed = forwardSpeed;
            this->reverseSpeed = reverseSpeed;
            forwardController.setSpeeds(forwardSpeed, 0);
            reverseController.setSpeeds(reverseSpeed, 0);
        }

        /**
         * Sets the durations for the forward and reverse motions.
         * @param forwardDuration The forward duration to set.
         * @param reverseDuration The reverse duration to set.
         */
        void setDurations(int forwardDuration, int reverseDuration)
        {
            this->forwardDuration = forwardDuration;
            this->reverseDuration = reverseDuration;
            forwardController.setDuration(forwardDuration);
            reverseController.setDuration(reverseDuration);
        }

    private:
        static constexpr double DEFAULT_FORWARD_SPEED = 1.4;  // %
        static constexpr double DEFAULT_REVERSE_SPEED = -0.8; // %
        static constexpr int DEFAULT_FORWARD_DURATION = 600;  // ms
        static constexpr int DEFAULT_REVERSE_DURATION = 700;  // ms

        // Systems
        TimeController forwardController;
        TimeController reverseController;
        BaseChassis &chassis;
        double forwardSpeed = DEFAULT_FORWARD_SPEED;
        double reverseSpeed = DEFAULT_REVERSE_SPEED;
        int forwardDuration = DEFAULT_FORWARD_DURATION;
        int reverseDuration = DEFAULT_REVERSE_DURATION;
    };
}