
#ifndef ODRIVEPROPERTIES_H
#define ODRIVEPROPERTIES_H

#include <inttypes.h>

struct Axis {

    int32_t current_state = 0;
	int32_t requested_state = 0;
    int32_t controller_config_input_mode = 0;

    bool min_endstop_config_enabled = 0;
    uint16_t min_endstop_config_gpio_num = 0;
    bool min_endstop_config_is_active_high = false;
    bool min_endstop_endstop_state = false;
    float min_endstop_config_offset = false;

    int32_t error = 0;
    int32_t motor_error = 0;
    int32_t encoder_error = 0;
    int32_t controller_error = 0;

	float controller_input_pos = 0;
    float encoder_pos_estimate = 0;
    float controller_config_pos_gain = 0;
    float controller_config_vel_gain = 0;
    float controller_config_vel_integrator_gain = 0;
    float controller_config_homing_speed = 0;
    bool controller_config_enable_overspeed_error = false;

    float trap_traj_config_vel_limit = 0;
    float trap_traj_config_accel_limit = 0;
    float trap_traj_config_decel_limit = 0;

	bool is_homed = false;
    bool motor_is_calibrated = false;
    bool motor_config_pre_calibrated = false;
    float motor_config_calibration_current = 0.f;
    float motor_config_current_lim = 0.f;
    float motor_config_torque_constant = 0.f;

    uint8_t clear_errors = 0;
};

struct ODriveProperties {

    uint8_t propertyIndex = 0;

    //   vvvv    Endpoints    vvvvv

	float vbus_voltage = 0;

    Axis axis0;
    Axis axis1;

};

#endif // ODRIVEPROPERTIES_H
