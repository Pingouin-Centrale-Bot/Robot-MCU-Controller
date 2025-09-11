#include "RemoteControl.h"
#include <string.h>
#include "config.h"

#include <btstack_port_esp32.h>

static const char *TAG = "RemoteControl";
static const double lift_max_h = 200;
static const double lift_min_h = 3;

struct uni_platform *get_robot_platform()
{
    static struct uni_platform plat = {
        .name = "robot_controller",
        .init = RemoteControl::platform_init,
        .on_init_complete = RemoteControl::platform_init_complete,
        .on_device_discovered = RemoteControl::platform_device_discovered,
        .on_device_connected = RemoteControl::platform_device_connected,
        .on_device_disconnected = RemoteControl::platform_device_disconnected,
        .on_device_ready = RemoteControl::platform_device_ready,
        .on_controller_data = RemoteControl::platform_controller_data,
        .get_property = RemoteControl::platform_get_property,
        .on_oob_event = RemoteControl::platform_oob_event,
    };
    return &plat;
}

Motion *RemoteControl::_motion = nullptr;
Lift *RemoteControl::_lift_left = nullptr;
Lift *RemoteControl::_lift_right = nullptr;

RemoteControl::RemoteControl(Motion *motion, Lift *lift_left, Lift *lift_right)
{
    ESP_LOGI(TAG, "Creating");
    _motion = motion;
    _lift_left = lift_left;
    _lift_right = lift_right;

    btstack_init();

    uni_platform_set_custom(get_robot_platform());
    uni_init(0, NULL);

    ESP_LOGI(TAG, "Created");
}

void RemoteControl::trigger_event_on_gamepad(uni_hid_device_t *d)
{
    if (d->report_parser.play_dual_rumble != NULL)
    {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 128 /* weak magnitude */,
                                          40 /* strong magnitude */);
    }
}

void RemoteControl::start()
{
    btstack_run_loop_execute();
}

RemoteControl::platform_instance_t *RemoteControl::get_my_platform_instance(uni_hid_device_t *d)
{
    return (platform_instance_t *)&d->platform_data[0];
}

void RemoteControl::platform_init(int argc, const char **argv)
{
}

void RemoteControl::platform_init_complete()
{
    ESP_LOGI(TAG, "Remote control platform initialised");

    // set allowlist for XboxOne controller
    bd_addr_t controller_addr;
    sscanf_bd_addr(BT_CONTROLLER_ADRESS, controller_addr);
    uni_bt_allowlist_remove_all();
    uni_bt_allowlist_add_addr(controller_addr); // Add to allowlist (stored in NVS)
    uni_bt_allowlist_set_enabled(true);         // Enable allowlist enforcement

    // Start scanning
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
}

void RemoteControl::platform_device_connected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Controller connected");
    uni_bt_stop_scanning_unsafe();
}

void RemoteControl::platform_device_disconnected(uni_hid_device_t *d)
{
    _motion->stop();
    _lift_left->stop_motor();
    _lift_left->disable_suction();
    _lift_left->disable_magnets();
    _lift_right->stop_motor();
    _lift_right->disable_suction();
    _lift_right->disable_magnets();

    uni_bt_start_scanning_and_autoconnect_unsafe();
    ESP_LOGI(TAG, "Controller disconnected");
}

uni_error_t RemoteControl::platform_device_ready(uni_hid_device_t *d)
{
    trigger_event_on_gamepad(d);
    return UNI_ERROR_SUCCESS;
}

uni_gamepad_t RemoteControl::_last_gamepad = {};

void RemoteControl::platform_controller_data(uni_hid_device_t *d, uni_controller_t *ctl)
{
    static uni_controller_t prev = {};
    uni_gamepad_t *gp;

    // Optimization to avoid processing the previous data so that the console
    // does not get spammed with a lot of logs, but remove it from your project.
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0)
    {
        return;
    }

    gp = &ctl->gamepad;
    ESP_LOGI(TAG, "btns: %d; ax_x: %" PRId32 "; ax_y: %" PRId32 "; brake: %" PRId32 "; throttle: %" PRId32 ";", gp->buttons, gp->axis_x, gp->axis_y, gp->brake, gp->throttle);
    
    // lift right controls
    if ((gp->buttons & BUTTON_B) && !(prev.gamepad.buttons & BUTTON_B)) // btn_b just got pressed
    {
        if (_lift_right->magnets_enabled())
        {
            _lift_right->disable_magnets();
            // d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 50 /* duration ms */, 0 /* weak magnitude */,
            //                                   50 /* strong magnitude */);
            xboxone_play_quad_rumble(d, 0, 50, 0, 255, 50, 20);
        }
        else
        {
            _lift_right->enable_magnets();
            // d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 0 /* weak magnitude */,
            //                                   100 /* strong magnitude */);
            xboxone_play_quad_rumble(d, 0, 150, 0, 255, 25, 10);
        }
    }
    if ((gp->buttons & BUTTON_SHOULDER_R) && !(prev.gamepad.buttons & BUTTON_SHOULDER_R)) // shoulder_r just got pressed
    {
        if (_lift_right->suction_enabled())
        {
            _lift_right->disable_suction();
        }
        else
        {
            _lift_right->enable_suction();
        }
    }
    if ((gp->buttons & BUTTON_Y) != (prev.gamepad.buttons & BUTTON_Y)) // btn_y changed state
    {
        if ((gp->buttons & BUTTON_Y) && !(gp->buttons & BUTTON_A)) // btn_y got pressed while btn_a not pressed
        {
            _lift_right->go_to(lift_max_h, false);
        }
        else if (!(gp->buttons & BUTTON_Y)) // btn_y lifted
        {
            _lift_right->stop_motor();
        }
    }
    if ((gp->buttons & BUTTON_A) != (prev.gamepad.buttons & BUTTON_A)) // btn_a changed state
    {
        if ((gp->buttons & BUTTON_A) && !(gp->buttons & BUTTON_Y)) // btn_a got pressed while btn_y not pressed
        {
            _lift_right->go_to(lift_min_h, false);
        }
        else if (!(gp->buttons & BUTTON_A)) // btn_a lifted
        {
            _lift_right->stop_motor();
        }
    }
    
    // lift left controls
    if ((gp->buttons & BUTTON_X) && !(prev.gamepad.buttons & BUTTON_X)) // btn_x just got pressed
    {
        if (_lift_left->magnets_enabled())
        {
            _lift_left->disable_magnets();
            // d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 50 /* duration ms */, 100 /* weak magnitude */,
            //                                   50 /* strong magnitude */);
            xboxone_play_quad_rumble(d, 0, 50, 255, 0, 50, 20);
        }
        else
        {
            _lift_left->enable_magnets();
            // d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 200 /* weak magnitude */,
            //                                   100 /* strong magnitude */);
            xboxone_play_quad_rumble(d, 0, 150, 255, 0, 25, 10);
        }
    }
    if ((gp->buttons & BUTTON_SHOULDER_L) && !(prev.gamepad.buttons & BUTTON_SHOULDER_L)) // shoulder_l just got pressed
    {
        if (_lift_left->suction_enabled())
        {
            _lift_left->disable_suction();
        }
        else
        {
            _lift_left->enable_suction();
        }
    }
    if ((gp->dpad & DPAD_UP) != (prev.gamepad.dpad & DPAD_UP)) // dpad_up changed state
    {
        if ((gp->dpad & DPAD_UP) && !(gp->dpad & DPAD_DOWN)) // dpad_up got pressed while dpad_down not pressed
        {
            _lift_left->go_to(lift_max_h, false);
        }
        else if (!(gp->dpad & DPAD_UP)) // dpad_up lifted
        {
            _lift_left->stop_motor();
        }
    }
    if ((gp->dpad & DPAD_DOWN) != (prev.gamepad.dpad & DPAD_DOWN)) // dpad_down changed state
    {
        if ((gp->dpad & DPAD_DOWN) && !(gp->dpad & DPAD_UP)) // dpad_down got pressed while dpad_up not pressed
        {
            _lift_left->go_to(lift_min_h, false);
        }
        else if (!(gp->dpad & DPAD_DOWN)) // dpad_down lifted
        {
            _lift_left->stop_motor();
        }
    }

    prev = *ctl;
}

void RemoteControl::platform_oob_event(uni_platform_oob_event_t event, void *data)
{
}
