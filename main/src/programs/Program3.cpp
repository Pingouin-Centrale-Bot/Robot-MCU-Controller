#include "Program3.h"
#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Program3::Program3(Motion* m, Lift* g, Lift* d, IHM* i, Lidar* l)
    : motion(m), liftG(g), liftD(d), ihm(i), lidar(l) {}

void Program3::init() {
}

void Program3::run() {
    motion->translate(450, 90, 200, 200, true);
    motion->translate(50, 0, 200, 200, true);
    motion->translate(70, 90, 30, 30, true);    // Collage robot
    motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
    motion->translate(80, 270, 200, 100, true); // S''eloigner
    motion->rotate(180, 150, 150);              // rotation

    motion->translate(85, 270, 200, 100, true); // collage robot 2
    motion->translate(50, 90, 200, 100, true);  // S''eloigner
    motion->translate(50, 0, 200, 200, true);   // Déplacement latéral

    liftD->go_to(130 - LIFT_0);
    liftD->enable_magnets();
    motion->translate(110, 270, 200, 100, true); // Avancer pour contct cannette
    vTaskDelay(pdMS_TO_TICKS(500));
    liftD->go_to(150 - LIFT_0);
    motion->translate(150, 90, 200, 100, true); // Reculer

    motion->rotate(180, 150, 150);              // rotation
    motion->translate(70, 90, 200, 100, true);  // Contact cannettes 2
    motion->translate(100, 0, 200, 200, true);  // Déplacement latéral
    motion->translate(70, 270, 200, 100, true); // Avancer pour contct cannette 2
    liftG->go_to(130 - LIFT_0);
    liftG->enable_magnets();
    vTaskDelay(pdMS_TO_TICKS(500));
    liftD->go_to(150 - LIFT_0);
    motion->translate(70, 270, 200, 100, true); // Reculer

    vTaskDelay(pdMS_TO_TICKS(5000));
    liftD->go_to(130 - LIFT_0); // En bas
    liftG->go_to(130 - LIFT_0); // En bas

}
