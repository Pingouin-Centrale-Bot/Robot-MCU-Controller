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
#include "programs/ProgramRemoteControl.h"
#include "init_robot.h"

static const char *TAG = "main";

bool emergency_stopped = false;

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
    liftG->calibrate(ihm);
    ihm->set_LED(1, 1);
    liftD->calibrate(ihm);
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
            case 7: currentProgram = new ProgramRemoteControl(motion, liftG, liftD, ihm, lidar); break;
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

    freeRobot(motion, liftG, liftD, ihm, lidar);

    ESP_LOGI(TAG, "Done");

    // WARNING: if program reaches end of function app_main() the MCU will
    // restart.
}

void waitTiretteAndStart()
{
    ihm->write_msg("Ready   ");
    //lidar->start_detection(); temp
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

    int prognum = __builtin_ctz(btns); // count trailing zeroes
    ihm->show_score(prognum);
    ihm->write_msg("Sel");
    vTaskDelay(pdMS_TO_TICKS(500));
    return prognum;
}