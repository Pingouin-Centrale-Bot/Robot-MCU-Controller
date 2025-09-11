#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "LedcStepper.h"
#include "IHM.h"
#include "config.h"
#include "TMC2209.h"

class Lift
{
public:
    Lift(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint8_t suction_pin, uint8_t magnet_pin, uint8_t microsteps, TMC2209::SerialAddress uart_adress, const char *tag);
    void enable_motor();
    void disable_motor();
    void disable_suction();
    void enable_suction();
    void disable_magnets();
    void enable_magnets();
    void go_to(double h, bool wait = true);
    void wait();
    double get_position();
    void reset_position();
    void reset_all();
    void calibrate(IHM *ihm);
    void stop_motor();
    bool suction_enabled() const { return _suction_enabled; }
    bool magnets_enabled() const { return _magnets_enabled; }

private:
    TMC2209 *driver;
    LedcStepper *_stepper;
    gpio_num_t _suction_pin;
    gpio_num_t _magnet_pin;
    int _steps_per_mm;
    int _speed_stp;
    const char *_tag;
    bool _suction_enabled = false;
    bool _magnets_enabled = false;
};