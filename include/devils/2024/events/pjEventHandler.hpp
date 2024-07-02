#pragma once

#include "../pepperJack.hpp"
#include "../../utils/eventHandler.hpp"

namespace devils
{
    class PJEventHandler : public EventHandler
    {
        PJEventHandler(PepperJack *robot) : robot(robot)
        {
        }

        /**
         * Runs a specific event on PJ.
         * @param eventName The name of the event to run.
         * @param params The parameters to pass to the event.
         */
        void runEvent(std::string eventName, std::string params[])
        {
            // Generic
            if (eventName == "pause" && params->size() == 1)
            {
                pros::delay(std::stoi(params[0]));
            }

            // Drivetrain
            else if (eventName == "rotateTo" && params->size() == 1)
            {
                double angle = std::stod(params[0]);
                Pose targetPose = Pose(0, 0, std::stod(params[0]));

                DirectController controller(robot->chassis, robot->imu);
                controller.setTargetPose(targetPose);
                controller.runSync();
            }

            else if (eventName == "driveDistance" && params->size() == 1)
            {
                double distance = std::stod(params[0]);
                // TODO: Implement
            }
            else if (eventName == "driveTime" && params->size() == 2)
            {
                double speed = std::stod(params[0]);
                double time = std::stod(params[1]);

                TimeController controller(robot->chassis, time, speed);
                controller.runSync();
            }

            // Manipulation
            else if (eventName == "intake")
            {
                robot->intake.intake();
            }
            else if (eventName == "outtake")
            {
                robot->intake.outtake();
            }
            else if (eventName == "leftWing")
            {
                robot->wings.extendLeft();
            }
            else if (eventName == "rightWing")
            {
                robot->wings.extendRight();
            }
            else if (eventName == "closeWings")
            {
                robot->wings.retractLeft();
                robot->wings.retractRight();
            }
            else if (eventName == "raiseLift")
            {
                robot->blocker.extend();
            }
            else if (eventName == "lowerLift")
            {
                robot->blocker.retract();
            }
            else if (eventName == "setSpeed" && params->size() == 1)
            {
                robot->chassis.setSpeed(std::stoi(params[0]), std::stoi(params[0]));
            }
            else if (eventName == "setSpeed" && params->size() == 2)
            {
                robot->chassis.setSpeed(std::stoi(params[0]), std::stoi(params[1]));
            }
        }

    private:
        PepperJack *robot;
    };
}