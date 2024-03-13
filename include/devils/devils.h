#pragma once

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

/*
 *      Headers defined in this file are accessible from anywhere in the project.
 */

// Autonomous
#include "control/pursuitController.hpp"
#include "control/linearController.hpp"
#include "control/collectionController.hpp"

// Chassis
#include "chassis/chassis.hpp"
#include "chassis/tankChassis.hpp"
#include "chassis/dummyChassis.hpp"

// Display
#include "display/display.hpp"
#include "display/odomRenderer.hpp"
#include "display/pathRenderer.hpp"
#include "display/controlRenderer.hpp"
#include "display/displayUtils.hpp"
#include "display/fieldRenderer.hpp"
#include "display/rectRenderer.hpp"
#include "display/polygonRenderer.hpp"
#include "display/gameObjectRenderer.hpp"
#include "display/statsRenderer.hpp"

// Hardware
#include "hardware/imu.hpp"
#include "hardware/opticalSensor.hpp"

// Odom
#include "odom/pose.hpp"
#include "odom/odomSource.hpp"
#include "odom/mergedOdom.hpp"
#include "odom/trackingWheelOdom.hpp"
#include "odom/differentialWheelOdom.hpp"

// Path
#include "path/pathGenerator.hpp"

// Game Object
#include "gameobject/gameObject.hpp"
#include "gameobject/gameObjectManager.hpp"

// Pros
#include "api.h"

// Robot
#include "robots/robot.hpp"

// Utils
#include "utils/curve.hpp"
#include "utils/pid.hpp"
#include "utils/eventTimer.hpp"
#include "utils/rect.hpp"
