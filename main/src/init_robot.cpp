#include "init_robot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "modules/IHM.h"
#include "modules/Lift.h"
#include "modules/Motion.h"
#include "modules/Lidar.h"
#include "config.h"

static const char *TAG = "initRobot";

void initRobot(Motion*& motion, Lift*& liftG, Lift*& liftD, IHM*& ihm, Lidar*& lidar) {
    ESP_LOGI(TAG, "START");

    motion = new Motion();
    ESP_LOGI(TAG, "Motion initialized");

    liftG = new Lift(M5_STP_PIN, M5_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE1_PIN, ELECTRO1_PIN, M5_MICROSTEP, M5_DRIVER_ADDRESS, "G_Lift");
    liftD = new Lift(M6_STP_PIN, M6_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE2_PIN, ELECTRO2_PIN, M6_MICROSTEP, M6_DRIVER_ADDRESS, "D_Lift");
    ESP_LOGI(TAG, "Lifts initialized");

    ihm = new IHM(CLK_IHM_PIN, DIO_IHM_PIN, STB_IHM_PIN);
    ESP_LOGI(TAG, "IHM initialized");

    lidar = new Lidar(BALISE_WIDTH, LIDAR_SERIAL, LIDAR_RX_PIN, motion);
    ESP_LOGI(TAG, "LiDAR initialized");
}

void freeRobot(Motion*& motion, Lift*& liftG, Lift*& liftD, IHM*& ihm, Lidar*& lidar) {
    delete motion;
    delete liftG;
    delete liftD;
    delete ihm;
    delete lidar;

    motion = nullptr;
    liftG = nullptr;
    liftD = nullptr;
    ihm = nullptr;
    lidar = nullptr;
}
