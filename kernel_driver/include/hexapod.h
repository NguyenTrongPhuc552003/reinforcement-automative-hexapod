#ifndef _HEXAPOD_H
#define _HEXAPOD_H

#include <linux/types.h>

// Hexapod configuration
#define NUM_LEGS                6
#define NUM_JOINTS_PER_LEG      3
#define TOTAL_SERVOS            (NUM_LEGS * NUM_JOINTS_PER_LEG)

// Joint identifiers
#define JOINT_COXA              0
#define JOINT_FEMUR             1
#define JOINT_TIBIA             2

// Leg identifiers
#define LEG_RIGHT_FRONT         0
#define LEG_RIGHT_MIDDLE        1
#define LEG_RIGHT_BACK          2
#define LEG_LEFT_FRONT          3
#define LEG_LEFT_MIDDLE         4
#define LEG_LEFT_BACK           5

// Device configuration
#define HEXAPOD_DEV_NAME        "hexapod"
#define HEXAPOD_CLASS_NAME      "hexapod_class"

#endif /* _HEXAPOD_H */
