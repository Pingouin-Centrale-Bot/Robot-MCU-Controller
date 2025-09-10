#include <stdio.h>
#include <cinttypes>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "driver/gpio.h"

#include "modules/IHM.h"
#include "modules/Lift.h"
#include "modules/Motion.h"
#include "modules/Lidar.h"
#include "config.h"
#include "programs/Program1.h"
#include "programs/Program2.h"
#include "programs/Program3.h"
#include "programs/Program4.h"
#include "init_robot.h"

static const char *TAG = "main";

void waitTiretteAndStart();
void calibrage();
int select_program();

int start = -1;
Motion *motion = NULL;
Lift *liftG = NULL;
Lift *liftD = NULL;
IHM *ihm = NULL;
Lidar *lidar = NULL;

void setup()
{
    initRobot(motion, liftG, liftD, ihm, lidar);
}

extern "C" void app_main(void)
{
    initArduino();
    setup();

    ihm->set_LED(0, 1);
    //liftG->calibrate(ihm); temp for SC4
    ihm->set_LED(1, 1);
    //liftD->calibrate(ihm); temp for SC4
    ihm->set_LED(2, 1);
    motion->calibrate(ihm);
    ihm->set_LED(3, 1);

    
    ProgramBase* currentProgram = nullptr;
    
    while (!currentProgram)
    {
        int prognum = select_program();
        switch(prognum) {
            case 0: currentProgram = new Program1(motion, liftG, liftD, ihm, lidar); break;
            case 1: currentProgram = new Program2(motion, liftG, liftD, ihm, lidar); break;
            case 2: currentProgram = new Program3(motion, liftG, liftD, ihm, lidar); break;
            case 3: currentProgram = new Program4(motion, liftG, liftD, ihm, lidar); break;
            default: 
                ihm->write_msg("Vide");
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }   
    }

    if(currentProgram) {
        currentProgram->init(); 
        waitTiretteAndStart();
        currentProgram->run(); 
        delete currentProgram;
    }


    // switch (btns)
    // {
    //     case 5:
    //     liftD->go_to(150 - LIFT_0, false);
    //     motion->translate(350, 90, 200, 100, true);
    //     motion->translate(365, 0, 200, 100, true);
    //     motion->translate(355, 270, 60, 40, true);
    //     ihm->show_score(4);
    //     motion->translate(405, 90, 200, 100, true);
    //     motion->translate(200, 0, 200, 100, true);
    //     motion->translate(700, 90, 200, 100, true);
    //     motion->translate(500, 180, 200, 100, true);
    //     motion->translate(1050, 263, 60, 40, true);
    //     ihm->show_score(8);
    //     motion->translate(200, 90, 200, 100, true);
    //     motion->translate(850, 0, 200, 100, true);
    //     motion->translate(1000, 90, 200, 100, true);
    //     ihm->show_score(18);

    //     break;
    // // case 5:
    // //     liftD->enable_magnets();
    // //     liftG->enable_magnets();
    // //     for (int i = 0; i < 10; i++)
    // //     {

    // //         liftD->go_to(130-LIFT_0); // En bas
    // //         liftG->go_to(130-LIFT_0); // En bas
    // //         vTaskDelay(pdMS_TO_TICKS(5000));
    // //         liftD->go_to(150-LIFT_0); // En bas
    // //         liftG->go_to(150-LIFT_0); // En bas
    // //         vTaskDelay(pdMS_TO_TICKS(5000));
    // //     }
    // //     liftD->disable_magnets();
    // //     liftG->disable_magnets();

    // //     break;
    // case 6:
    //     liftD->go_to(155 - LIFT_0, false);
    //     motion->translate(350, 90, 200, 100, true);
    //     motion->translate(365, 0, 200, 100, true);
    //     motion->translate(355, 270, 60, 40, true);
    //     ihm->show_score(4);
    //     motion->translate(405, 90, 200, 100, true);
    //     motion->translate(400, 65, 200, 100, true);
    //     motion->rotate(720, 75, 50);
    //     motion->rotate(-720, 75, 50);
    //     motion->translate(650, 65, 200, 100, true);
    //     ihm->show_score(14);
    //     break;
    // case 7:
    //     liftD->go_to(155 - LIFT_0, false);
    //     motion->translate(350, 90, 200, 100, true);
    //     motion->translate(365, 180, 200, 100, true);
    //     motion->translate(355, 270, 60, 40, true);
    //     ihm->show_score(4);
    //     motion->translate(405, 90, 200, 100, true);
    //     motion->translate(400, 115, 200, 100, true);
    //     motion->rotate(-720, 75, 50);
    //     motion->rotate(720, 75, 50);
    //     motion->translate(650, 115, 200, 100, true);
    //     ihm->show_score(14);
    //     break;
    // default:
    //     break;
    // }

    // while (1) {
    //     liftD->go_to(80, false);
    //     liftG->go_to(80);
    //     liftD->wait();
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     liftD->disable_suction();
    //     liftD->disable_magnets();
    //     liftG->disable_suction();
    //     liftG->disable_magnets();

    //     liftD->go_to(10, false);
    //     liftG->go_to(40);
    //     liftG->enable_suction();
    //     liftG->enable_magnets();
    //     liftD->wait();
    //     liftD->enable_suction();
    //     liftD->enable_magnets();
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    freeRobot(motion, liftG, liftD, ihm, lidar);

    ESP_LOGI(TAG, "Done");

    // WARNING: if program reaches end of function app_main() the MCU will
    // restart.
}

void waitTiretteAndStart()
{
    ihm->write_msg("Ready   ");
    lidar->start_detection();
    gpio_reset_pin((gpio_num_t)TIRETTE_PIN);
    gpio_set_direction((gpio_num_t)TIRETTE_PIN, GPIO_MODE_INPUT);
    while (!gpio_get_level((gpio_num_t)TIRETTE_PIN))
    {
        while (!gpio_get_level((gpio_num_t)TIRETTE_PIN))
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    start = millis() + 100;
    // ihm->show_progress_live(start);
    ihm->show_score(0);
}

void calibrage()
{
    motion->translate(100, 90);
}

int select_program() 
{
    ihm->write_msg("Prog.    ");
    uint8_t btns = ihm->wait_get_buttons();

    int prognum = 0;
    while ((btns & 1) == 0)
    {
        btns >>= 1;
        ++prognum;
    }
    ihm->show_score(prognum);
    ihm->write_msg("Sel");
    return prognum;
}