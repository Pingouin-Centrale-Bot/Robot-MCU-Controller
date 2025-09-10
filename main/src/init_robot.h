#ifndef INIT_H
#define INIT_H

#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"

void initRobot(Motion*& motion, Lift*& liftG, Lift*& liftD, IHM*& ihm, Lidar*& lidar);
void freeRobot(Motion*& motion, Lift*& liftG, Lift*& liftD, IHM*& ihm, Lidar*& lidar);

#endif