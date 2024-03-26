#pragma once
#include "pros/motors.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include "../utils/ramp.hpp"
#include "../geometry/perspective.hpp"
#include "../gameobject/gameObject.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a vision sensor object. All events are logged.
     */
    class VisionSensor
    {
    public:
        // Thank you James Pearman for these measurements
        // https://www.vexforum.com/t/vision-sensor-fov-measurements/62397
        static constexpr int VISION_WIDTH = VISION_FOV_WIDTH;   // px
        static constexpr int VISION_HEIGHT = VISION_FOV_HEIGHT; // px
        static constexpr int VISION_WIDTH_FOV = 61;             // degrees
        static constexpr int VISION_HEIGHT_FOV = 41;            // degrees

        /**
         * Creates a vision sensor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor (from 1 to 21)
         */
        VisionSensor(std::string name, uint8_t port)
            : name(name),
              sensor(port, pros::E_VISION_ZERO_CENTER)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": port is invalid");
        }

        /**
         * Gets any objects detected by the vision sensor.
         * @return The objects detected by the vision sensor
         */
        std::vector<DisplayPoint> getObjects()
        {
            std::vector<DisplayPoint> objects = {};

            // Get object count
            int objectCount = sensor.get_object_count();
            if (objectCount == 0)
                return objects;

            // Handle Errors
            if (objectCount == PROS_ERR)
            {
                if (LOGGING_ENABLED)
                    Logger::error(name + ": failed to get object count");
                return objects;
            }

            // Get objects
            for (int i = 0; i < objectCount; i++)
            {
                // Get Object
                pros::vision_object_s_t object = sensor.get_by_size(i);
                if (object.signature == 0)
                    continue;

                // Filter out small objects
                if (object.width * object.height < MIN_OBJECT_AREA)
                    continue;

                // Create Display Object
                short x = object.x_middle_coord;
                short y = object.y_middle_coord;
                DisplayPoint displayObject = DisplayPoint{(double)x, (double)y};
                objects.push_back(displayObject);
            }

            return objects;
        }

        /**
         * Sets the vision sensor's LED color.
         * Overrides the default LED behavior.
         * @param color The color to set the LED to
         */
        void setLEDColor(int32_t color)
        {
            auto status = sensor.set_led(color);
            if (status == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": failed to set LED color");
        }

        /**
         * Resets the vision sensor's LED color to the default behavior.
         */
        void resetLEDColor()
        {
            auto status = sensor.clear_led();
            if (status == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": failed to reset LED color");
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;
        static constexpr double MIN_OBJECT_AREA = 10 * 10; // px^2

        std::string name;
        pros::Vision sensor;
    };
}