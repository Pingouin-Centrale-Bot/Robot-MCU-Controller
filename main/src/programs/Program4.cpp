#include "Program4.h"
#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Program4::Program4(Motion* m, Lift* g, Lift* d, IHM* i, Lidar* l)
    : motion(m), liftG(g), liftD(d), ihm(i), lidar(l) {}

void Program4::init() {
}

void Program4::run() {
    liftD->go_to(150 - LIFT_0, false);
    motion->translate(350, 90, 200, 100, true);
    motion->translate(365, 180, 200, 100, true);
    motion->translate(355, 270, 60, 40, true);
    ihm->show_score(4);
    motion->translate(405, 90, 200, 100, true);
    motion->translate(200, 180, 200, 100, true);
    motion->translate(700, 90, 200, 100, true);
    motion->translate(500, 0, 200, 100, true);
    motion->translate(1050, 277, 60, 40, true);
    ihm->show_score(8);
    motion->translate(200, 90, 200, 100, true);
    motion->translate(850, 180, 200, 100, true);
    motion->translate(1000, 90, 200, 100, true);
    ihm->show_score(18);
}
