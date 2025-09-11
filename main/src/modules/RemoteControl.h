#pragma once

#include "Motion.h"
#include "Lift.h"
#include "Battery.h"

#include <uni.h>

class RemoteControl
{
public:
    RemoteControl(Motion *motion, Lift *lift_left, Lift *lift_right, Battery *battery);
    static void trigger_event_on_gamepad(uni_hid_device_t* d);
    static void start();

private:
    static Motion *_motion;
    static Lift *_lift_left;
    static Lift *_lift_right;
    static Battery *_battery;

    typedef struct platform_instance_s {
        uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
    } platform_instance_t;

    static platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

    static void platform_init(int argc, const char** argv);
    static void platform_init_complete();
    static uni_error_t platform_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
        return UNI_ERROR_SUCCESS;
    };
    static void platform_device_connected(uni_hid_device_t* d);
    static void platform_device_disconnected(uni_hid_device_t* d);
    static uni_error_t platform_device_ready(uni_hid_device_t* d);
    static const uni_property_t* platform_get_property(uni_property_idx_t idx) {
        return NULL;
    };
    static void platform_controller_data(uni_hid_device_t* d, uni_controller_t* ctl);
    static void platform_oob_event(uni_platform_oob_event_t event, void* data);

    friend struct uni_platform* get_robot_platform();

    static uni_gamepad_t _last_gamepad;
};
