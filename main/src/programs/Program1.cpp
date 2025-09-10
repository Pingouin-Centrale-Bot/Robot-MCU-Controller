#include "Program1.h"
#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Program1::Program1(Motion* m, Lift* g, Lift* d, IHM* i, Lidar* l)
    : motion(m), liftG(g), liftD(d), ihm(i), lidar(l) {}

void Program1::init() {
}

void Program1::run() {
    // le code de ton ancien case 0
    motion->translate(650, 90, 200, 100, true);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        liftG->enable_suction();
        liftG->enable_magnets();
        vTaskDelay(pdMS_TO_TICKS(5000));
        motion->rotate(180, 75, 50);
        motion->translate(650, 90, 200, 100, true);
        vTaskDelay(pdMS_TO_TICKS(1000));
        liftG->disable_magnets();
        liftG->disable_suction();
        vTaskDelay(pdMS_TO_TICKS(2000));
        motion->translate(100, 270, 200, 100, true);
        motion->rotate(-90-45, 75, 50);
        motion->translate(450, -45+180, 200, 100, true);
        motion->rotate(-45, 75, 50);
        motion->translate(100, 90, 200, 100, true);
    }

    motion->translate(450, 90, 200, 200, true);
    motion->translate(50, 0, 200, 200, true);
    liftG->go_to(145 - LIFT_0);              // Lever large
    motion->translate(70, 90, 30, 30, true); // Collage robot
    liftG->go_to(130 - LIFT_0);              // Contact ventouse
    liftG->enable_suction();
    vTaskDelay(pdMS_TO_TICKS(500));
    liftG->go_to(142 - LIFT_0);                 // Lever planche
    motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
    liftG->go_to(170 - LIFT_0);                 // Lever planche
    motion->translate(80, 270, 200, 100, true); // S''eloigner
    motion->rotate(180, 150, 150);              // rotation

    vTaskDelay(pdMS_TO_TICKS(1000));
    // Ca marche
    liftD->go_to(145 - LIFT_0);                 // Lever large
    motion->translate(85, 270, 200, 100, true); // collage robot 2
    liftD->go_to(130 - LIFT_0);                 // Contact ventouse 2
    liftD->enable_suction();
    vTaskDelay(pdMS_TO_TICKS(1000));
    liftD->go_to(180 - LIFT_0);                // Lever planche 2
    motion->translate(50, 90, 200, 100, true); // S''eloigner
    motion->translate(50, 0, 200, 200, true);  // Déplacement latéral

    liftD->go_to(130 - LIFT_0);
    liftD->enable_magnets();
    motion->translate(110, 270, 200, 100, true); // Avancer pour contct cannette
    liftD->go_to(135 - LIFT_0);
    vTaskDelay(pdMS_TO_TICKS(500));
    motion->translate(150, 90, 200, 100, true); // Reculer

    motion->rotate(180, 150, 150);              // rotation
    motion->translate(70, 90, 200, 100, true);  // Contact cannettes 2
    motion->translate(100, 0, 200, 200, true);  // Déplacement latéral
    motion->translate(70, 270, 200, 100, true); // Avancer pour contct cannette 2
    liftG->go_to(155 - LIFT_0);
    liftG->enable_magnets();
    vTaskDelay(pdMS_TO_TICKS(500));
    motion->translate(70, 270, 200, 100, true); // Reculer

    vTaskDelay(pdMS_TO_TICKS(5000));
    liftD->go_to(130 - LIFT_0); // En bas
    liftG->go_to(130 - LIFT_0); // En bas

    liftD->disable_magnets();
    liftG->disable_magnets();
    liftG->disable_suction();
    liftD->disable_suction();
}
