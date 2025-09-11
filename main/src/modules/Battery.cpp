#include "Battery.h"

#include "config.h"

void monitor_task(void *pvParameter)
{
    Battery *battery = static_cast<Battery *>(pvParameter);
    while (1)
    {
        float voltage = battery->get_voltage();
        if (voltage < 21)
        {
            battery->consecutive_low_bat++;
            if (battery->consecutive_low_bat > 2)
            {
                battery->emergency_stop();
                if (battery->_ihm)
                    battery->_ihm->write_msg("LOW BATT");
            }
        }
        else
        {
            battery->consecutive_low_bat = 0;
            if (battery->_ihm)
            {
                char msg[16];
                snprintf(msg, sizeof(msg), "%.3f v ", voltage);
                battery->_ihm->write_msg(msg);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

Battery::Battery(Motion *m, Lift *g, Lift *d)
    : _motion(m), _liftG(g), _liftD(d)
{
    _sense_pin = (gpio_num_t)V_BATT_PIN;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &_adc_handle));
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc_handle, _adc_channel, &config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &_adc_cali_handle));
}

float Battery::get_voltage()
{
    int raw = 0;

    ESP_LOGI("Battery", "reading voltage");
    ESP_ERROR_CHECK(adc_oneshot_read(_adc_handle, _adc_channel, &raw));

    int voltage_mv = raw; // default if no calibration
    if (_adc_cali_handle)
    {
        adc_cali_raw_to_voltage(_adc_cali_handle, raw, &voltage_mv);
    }

    float voltage = (voltage_mv / 1000.0f) * V_BATT_RATIO;
    return voltage;
}

void Battery::start_monitoring(IHM *ihm)
{
    _ihm = ihm;
    if (!_monitoring_task_handle)
    {
        xTaskCreate(monitor_task, "battery_monitorTask", 8192 * 4, (void *)this, PRIORITY_BATTERY_MONITOR, &_monitoring_task_handle);
    }
}

void Battery::enable_emergency_trigger(int pin)
{

}

void Battery::emergency_stop()
{
    emergency_stopped = true;
    _liftD->disable_magnets();
    _liftD->disable_suction();
    _liftG->disable_magnets();
    _liftG->disable_suction();

    _motion->stop();
    _motion->disable_motors();

    _liftG->stop_motor();
    _liftG->disable_motor();
    _liftD->stop_motor();
    _liftD->disable_motor();
}
