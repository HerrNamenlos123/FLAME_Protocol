
#ifndef ODRIVEPROPERTIES_H
#define ODRIVEPROPERTIES_H

#include <inttypes.h>

// AXIS STATE
#define AXIS_STATE_UNDEFINED 0
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_STARTUP_SEQUENCE 2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3
#define AXIS_STATE_MOTOR_CALIBRATION 4
#define AXIS_STATE_ENCODER_INDEX_SEARCH 6
#define AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define AXIS_STATE_LOCKIN_SPIN 9
#define AXIS_STATE_ENCODER_DIR_FIND 10
#define AXIS_STATE_HOMING 11
#define AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION 12
#define AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION 13

// INPUT MODE
#define INPUT_MODE_INACTIVE 0
#define INPUT_MODE_PASSTHROUGH 1
#define INPUT_MODE_VEL_RAMP 2
#define INPUT_MODE_POS_FILTER 3
#define INPUT_MODE_MIX_CHANNELS 4
#define INPUT_MODE_TRAP_TRAJ 5
#define INPUT_MODE_TORQUE_RAMP 6
#define INPUT_MODE_MIRROR 7
#define INPUT_MODE_TUNING 8

struct ODriveProperties {

    uint8_t propertyIndex = 0;

    //   vvvv    Endpoints    vvvvv

	float vbus_voltage = 0;

	int32_t axis0_current_state = 0;
	int32_t axis0_requested_state = 0;
    int32_t axis0_controller_config_input_mode = 0;
	bool axis0_is_homed = false;

    bool axis0_min_endstop_config_enabled = 0;
    bool axis0_min_endstop_endstop_state = false;

    int32_t axis0_error = 0;
    int32_t axis0_motor_error = 0;
    int32_t axis0_encoder_error = 0;
    int32_t axis0_controller_error = 0;

	float axis0_controller_input_pos = 0;
    float axis0_encoder_pos_estimate = 0;

    bool axis0_motor_is_calibrated = false;

    uint8_t axis0_clear_errors = 0;



	int32_t axis1_current_state = 0;
	int32_t axis1_requested_state = 0;
    int32_t axis1_controller_config_input_mode = 0;
	bool axis1_is_homed = false;

    bool axis1_min_endstop_config_enabled = 0;
    bool axis1_min_endstop_endstop_state = false;

    int32_t axis1_error = 0;
    int32_t axis1_motor_error = 0;
    int32_t axis1_encoder_error = 0;
    int32_t axis1_controller_error = 0;

	float axis1_controller_input_pos = 0;
    float axis1_encoder_pos_estimate = 0;

    bool axis1_motor_is_calibrated = false;

    uint8_t axis1_clear_errors = 0;

};

#endif // ODRIVEPROPERTIES_H
