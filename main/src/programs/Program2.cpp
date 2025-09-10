#include "Program2.h"
#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Program2::Program2(Motion* m, Lift* g, Lift* d, IHM* i, Lidar* l)
    : motion(m), liftG(g), liftD(d), ihm(i), lidar(l) {}

void Program2::init() {
}

void Program2::run() {
    // liftG->go_to(145-LIFT_0); // Lever large
    // motion->translate(450, 90, 200, 200, true);
    // liftG->go_to(130-LIFT_0); // Contact ventouse
    // motion->translate(50, 0, 200, 200, true);
    // motion->translate(70, 90, 30, 30, true); // Collage robot
    // liftG->enable_suction();
    // vTaskDelay(pdMS_TO_TICKS(500));
    // liftG->go_to(142-LIFT_0); // Lever planche
    // motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
    // liftG->go_to(170-LIFT_0); // Lever planche
    // motion->translate(80, 270, 200, 100, true); // S''eloigner
    // motion->rotate(180, 150, 150); // rotation

    liftG->go_to(145 - LIFT_0, false); // Lever large
    motion->translate(450, 90, 200, 200, true);
    liftG->wait();
    motion->translate(50, 0, 200, 200, true);
    motion->translate(70, 90, 30, 30, true); // Collage robot
    liftG->go_to(135 - LIFT_0);              // Contact ventouse
    liftG->enable_suction();
    vTaskDelay(pdMS_TO_TICKS(500));
    liftG->go_to(142 - LIFT_0);                 // Lever planche
    motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
    liftG->go_to(170 - LIFT_0, false);          // Lever planche
    motion->translate(80, 270, 200, 100, true); // S''eloigner
    motion->rotate(180, 150, 150);              // rotation

    vTaskDelay(pdMS_TO_TICKS(5000));
}
