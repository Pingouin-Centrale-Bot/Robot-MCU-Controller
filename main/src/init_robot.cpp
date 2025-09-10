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

    // Initializing UART coms for steppers
    M_DRIVE_SERIAL.begin(115200, SERIAL_8N1, M_DRIVE_RX, M_DRIVE_TX);
    M_CHARIOT_SERIAL.begin(115200, SERIAL_8N1, M_CHARIOT_RX, M_CHARIOT_TX);
    vTaskDelay(pdMS_TO_TICKS(100)); // for the UART com to settle (not proved to be usefull, but if it aint brock dont fix it)
    // gpio_reset_pin((gpio_num_t)M_CHARIOT_RX);
    // gpio_set_direction((gpio_num_t)M_CHARIOT_RX, GPIO_MODE_OUTPUT);
    // gpio_set_level((gpio_num_t)M_CHARIOT_RX, 0);
    // gpio_reset_pin((gpio_num_t)M_CHARIOT_TX);
    // gpio_set_direction((gpio_num_t)M_CHARIOT_TX, GPIO_MODE_OUTPUT);
    // gpio_set_level((gpio_num_t)M_CHARIOT_TX, 0);

    ESP_LOGI(TAG, "Motor serial initialized");

    motion = new Motion();
    ESP_LOGI(TAG, "Motion initialized");

    liftG = new Lift(M5_STP_PIN, M5_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE1_PIN, ELECTRO1_PIN, M5_MICROSTEP, 'G');
    liftD = new Lift(M6_STP_PIN, M6_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE2_PIN, ELECTRO2_PIN, M6_MICROSTEP, 'D');
    ESP_LOGI(TAG, "Lifts initialized");

    ihm = new IHM(CLK_IHM_PIN, DIO_IHM_PIN, STB_IHM_PIN);
    ESP_LOGI(TAG, "IHM initialized");

    // Ld19p = new ld19p(LIDAR_SERIAL, LIDAR_RX_PIN);
    // ESP_LOGI(TAG, "Ld19p initialized");

    lidar = new Lidar(BALISE_WIDTH, LIDAR_SERIAL, LIDAR_RX_PIN, motion);
    ESP_LOGI(TAG, "LiDAR initialized");

    // to change

    #define STEPS_HZ 200 * 5
    #define ACCEL_TIME_S 1

    // stepper1->setSpeedInHz(STEPS_HZ * M1_MICROSTEP);
    // stepper1->setAcceleration(STEPS_HZ * M1_MICROSTEP / ACCEL_TIME_S);
    // stepper1->attachToPulseCounter(); // Needed, because getCurrentPosition() is not precise when using RMT driver (wich is mandatory with esp-idf=5)

    // stepper2->setSpeedInHz(STEPS_HZ * M2_MICROSTEP);
    // stepper2->setAcceleration(STEPS_HZ * M2_MICROSTEP / ACCEL_TIME_S);
    // stepper2->attachToPulseCounter();

    // stepper3->setDirectionPin(M3_DIR_PIN);
    // stepper3->setSpeedInHz(STEPS_HZ * M3_MICROSTEP);
    // stepper3->setAcceleration(STEPS_HZ * M3_MICROSTEP / ACCEL_TIME_S);
    // //stepper3->attachToPulseCounter();

    // stepper4->setDirectionPin(M4_DIR_PIN);
    // stepper4->setSpeedInHz(STEPS_HZ * M4_MICROSTEP);
    // stepper4->setAcceleration(STEPS_HZ * M4_MICROSTEP / ACCEL_TIME_S);
    // //stepper4->attachToPulseCounter();
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
