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

    while (1) {
        motion->translate_velocity(45, 100);
        vTaskDelay(pdMS_TO_TICKS(5000));
        motion->translate_velocity(45+180, 100);
        vTaskDelay(pdMS_TO_TICKS(5000));
    };

}
