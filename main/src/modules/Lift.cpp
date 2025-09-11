#include "Lift.h"

Lift::Lift(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint8_t suction_pin, uint8_t magnet_pin, uint8_t microsteps, TMC2209::SerialAddress uart_adress, const char *tag)
{
    _tag = tag;

    _steps_per_mm = (M_CHARIOT_STEPS_PER_TURN * microsteps) / M_CHARIOT_TRAVEL_PER_TURN;
    ESP_LOGI(_tag, "_steps_per_mm: %d", _steps_per_mm);
    _speed_stp = M_CHARIOT_SPEED_MM * _steps_per_mm;
    driver = new TMC2209();
    driver->setup(M_CHARIOT_SERIAL, 115200, uart_adress, M_CHARIOT_RX, M_CHARIOT_TX);
    driver->setReplyDelay(4);
    driver->enableAutomaticCurrentScaling();
    driver->enableAutomaticGradientAdaptation();
    driver->setRMSCurrent(M_CHARIOT_CURRENT_MA, M_R_SENSE);
    driver->setMicrostepsPerStep(microsteps);
    driver->enable();

    _stepper = new LedcStepper(step_pin, dir_pin, en_pin, true, PRIORITY_LIFT_STEPPERS, PRIORITY_LIFT_STOP_ISR);

    _suction_pin = (gpio_num_t)suction_pin;
    gpio_reset_pin(_suction_pin);
    gpio_set_direction(_suction_pin, GPIO_MODE_OUTPUT);
    disable_suction();

    _magnet_pin = (gpio_num_t)magnet_pin;
    gpio_reset_pin(_magnet_pin);
    gpio_set_direction(_magnet_pin, GPIO_MODE_OUTPUT);
    disable_magnets();
}

void Lift::enable_motor()
{
    if (!emergency_stopped)
        _stepper->enable();
}

void Lift::disable_motor()
{
    _stepper->disable();
}

void Lift::disable_suction()
{
    if (_suction_enabled)
    {
        gpio_set_level(_suction_pin, 0);
        _suction_enabled = false;
        ESP_LOGI(_tag, "Suction disabled");
    }
}

void Lift::enable_suction()
{
    if (!_suction_enabled)
    {
        gpio_set_level(_suction_pin, 1);
        _suction_enabled = true;
        ESP_LOGI(_tag, "Suction enabled");
    }
}

void Lift::disable_magnets()
{
    if (_magnets_enabled)
    {
        gpio_set_level(_magnet_pin, 0);
        _magnets_enabled = false;
        ESP_LOGI(_tag, "Magnets disabled");
    }
}

void Lift::enable_magnets()
{
    if (!_magnets_enabled)
    {
        gpio_set_level(_magnet_pin, 1);
        _magnets_enabled = true;
        ESP_LOGI(_tag, "Magnets enabled");
    }
}

void Lift::go_to(double h, bool wait)
{
    ESP_LOGI(_tag, "Going to %fmm, %fsteps", h, h * _steps_per_mm);
    if (h < 0)
    {
        ESP_LOGE(_tag, "Lift height negative: %f", h);
        return;
    }
    _stepper->go_to(h * _steps_per_mm, _speed_stp, wait);
}

void Lift::wait()
{
    _stepper->wait();
}

double Lift::get_position()
{
    return _stepper->get_position() / _steps_per_mm;
}

void Lift::reset_position()
{
    _stepper->reset_position();
}

void Lift::reset_all()
{
    _stepper->disable();
    disable_magnets();
    disable_suction();
}

void Lift::calibrate(IHM *ihm)
{
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_CAL_LIFT);

    ihm->write_msg((String("c l") + _tag[0] + " D V").c_str());
    driver->setRMSCurrent(M_CHARIOT_CAL_CURRENT_MA, M_R_SENSE);
    enable_motor();
    ESP_LOGI(_tag, "Calibration started");

    bool goingDown = false;
    while (1)
    {
        ESP_LOGI(_tag, "_steps_per_mm: %d", _steps_per_mm);
        uint8_t btns = ihm->get_buttons();
        ESP_LOGI(_tag, "Cal: btns pressed: %d", btns);
        if (btns & (uint8_t)(1 << (8 - 1)))
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // debounce delay
            if (!(ihm->get_buttons() & (uint8_t)(1 << (8 - 1))))
                continue;
            if (goingDown)
                stop_motor();
            ESP_LOGI(_tag, "Calibration accepted");
            ihm->write_msg("accepted");
            vTaskDelay(pdMS_TO_TICKS(100));
            driver->setRMSCurrent(M_CHARIOT_CURRENT_MA, M_R_SENSE);
            reset_position();
            ESP_LOGI(_tag, "Calibration saved");
            go_to(10);
            vTaskDelay(pdMS_TO_TICKS(500));
            ihm->write_msg("");
            vTaskPrioritySet(NULL, prvPriority);
            ESP_LOGI(_tag, "Ended calibration");
            return;
        }
        else if (btns & (uint8_t)(1 << (6 - 1)))
        {
            if (!goingDown)
            {
                _stepper->step_speed(-(M_CHARIOT_CAL_SPEED * _steps_per_mm));
                goingDown = true;
                ESP_LOGI(_tag, "Cal: Started going down");
            }
        }
        else if (goingDown)
        {
            stop_motor();
            goingDown = false;
            ESP_LOGI(_tag, "Cal: Stopped going down");
        }

        ESP_LOGI(_tag, "Cal: Started waiting");
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGI(_tag, "Cal: Done waiting");
    }
}

void Lift::stop_motor()
{
    _stepper->stop();
}
