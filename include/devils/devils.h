#pragma once

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

/*
 *      Headers defined in this file are accessible from anywhere in the project.
 */

// Autonomous
#include "control/pursuitController.hpp"
#include "control/linearController.hpp"

// Chassis
#include "chassis/chassis.hpp"
#include "chassis/tankChassis.hpp"
#include "chassis/dummyChassis.hpp"

// Display
#include "display/display.hpp"
#include "display/odomRenderer.hpp"
#include "display/motionRenderer.hpp"
#include "display/controlRenderer.hpp"
#include "display/displayUtils.hpp"
#include "display/fieldRenderer.hpp"
#include "display/rectRenderer.hpp"
#include "display/gameObjectRenderer.hpp"
#include "display/statsRenderer.hpp"

// Hardware
#include "hardware/imu.hpp"
#include "hardware/opticalSensor.hpp"

// Odom
#include "odom/tankWheelOdometry.hpp"
#include "odom/dummyOdometry.hpp"

// Path
#include "path/splineGenerator.hpp"
#include "path/squigglesGenerator.hpp"
#include "path/linearGenerator.hpp"

// Game Object
#include "gameobject/gameobject.hpp"

// Pros
#include "api.h"

// Robot
#include "robots/robot.hpp"

// Systems
#include "systems/blockerSystem.hpp"
#include "systems/intakeSystem.hpp"
#include "systems/actuateIntakeSystem.hpp"
#include "systems/launcherSystem.hpp"
#include "systems/wingSystem.hpp"
#include "systems/climbSystem.hpp"
#include "systems/ledSystem.hpp"

// Utils
#include "utils/curve.hpp"
#include "utils/pid.hpp"
#include "utils/autoTimer.hpp"
#include "utils/rect.hpp"
