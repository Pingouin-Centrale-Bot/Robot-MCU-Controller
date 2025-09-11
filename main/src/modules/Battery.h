#pragma once

#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "esp_adc/adc_oneshot.h"

class Battery
{
    friend void monitor_task(void *pvParameter);

public:
    Battery(Motion *m, Lift *g, Lift *d);
    float get_voltage();
    void start_monitoring(IHM *ihm = nullptr);
    void enable_emergency_trigger(int pin);
    void emergency_stop();

private:
    TaskHandle_t _monitoring_task_handle = NULL;
    gpio_num_t _sense_pin = GPIO_NUM_NC;
    gpio_num_t _emergency_trigger_pin = GPIO_NUM_NC;
    adc_oneshot_unit_handle_t _adc_handle;
    adc_cali_handle_t _adc_cali_handle;
    static const adc_channel_t _adc_channel = ADC_CHANNEL_0;
    Motion *_motion;
    Lift *_liftG;
    Lift *_liftD;
    IHM *_ihm;
    uint8_t consecutive_low_bat = 0;
};