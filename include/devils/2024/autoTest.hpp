#pragma once
#include "../devils.h"
#include "systems/pjAutoController.hpp"
#include <time.h>

#define __ARM_NEON
#include "incbin/incbin.h"

namespace devils
{
    /**
     * Represents an autonomous test robot
     */
    struct AutoTest : public Robot
    {
        /**
         * Creates a new instance of Auto Tester.
         */
        AutoTest()
        {
            // Run Display
            displayStack.runAsync();
        }

        void opcontrol() override
        {

            // Loop
            while (true)
            {
                // Delay
                pros::delay(20);
            }
        }

    private:
        // Display
        Display displayStack = Display({new PathRenderer(nullptr),
                                        new GridRenderer(),
                                        new FieldRenderer(),
                                        new PathPickerRenderer({"None", "Red", "Blue", "Skills"}),
                                        new StatsRenderer()});
        StatsRenderer *statsRenderer = displayStack.getRenderer<StatsRenderer>();
    };
}