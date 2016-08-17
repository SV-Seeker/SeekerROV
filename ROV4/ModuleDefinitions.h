#pragma once
#include "AConfig.h"

// #include "PluginConfig.h"

// ---------------------------------------
// Conditional module definitions
// -------------------------------
// Once these objects are instantiated, they will register themselves as modules in the module manager.
// They can also be accessed individually as namespace members.
// ---------------------------------------

#if(HAS_BNO055)
#define COMPASS_ENABLED 1
#define GYRO_ENABLED 1
#define ACCELEROMETER_ENABLED 1
#include "CBNO055.h"
CBNO055 m_boschIMU;
#endif
