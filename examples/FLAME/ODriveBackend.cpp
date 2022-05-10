
#include "ODriveBackend.h"
#include "ODriveProperties.h"
#include "endpoints.h"

#define WRITE(var, endpoint) odrv->sendWriteRequest<endpoint>(var)

#define TRIGGER_RETURN(var, default, endpoint) \
    if (var != default) { \
        odrv->sendWriteRequest<endpoint>(var); \
        var = default; \
        return; \
    }

#define FUNCTION(var, endpoint) \
    while (var) { \
        odrv->sendWriteRequest<endpoint>(true); \
        var--; \
    } 

#define READ(endpoint) odrv->sendReadRequest<endpoint>()

#define RUN_EVERY_NTH_CYCLE_START() int _idx = 0;
#define RUN_EVERY_NTH_CYCLE(exec) if (prop->propertyIndex == _idx++) { exec; }
#define RUN_EVERY_NTH_CYCLE_END() \
    prop->propertyIndex++; \
    if (prop->propertyIndex >= _idx++) \
        prop->propertyIndex = 0;


ODriveProperties odrvData0;
ODriveProperties odrvData1;
uint32_t lastUpdate = 0;
uint32_t cycleTime = 10000;

void OnDataCallback(ODriveProperties* odrv, uint16_t endpointID, uint8_t* data, size_t dataSize) {

    switch (endpointID) {
        ENDPOINT_CASE(odrv->vbus_voltage, ENDPOINT_VBUS_VOLTAGE); break;

        // Axis 0
        ENDPOINT_CASE(odrv->axis0.encoder_pos_estimate, ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE); break;
        ENDPOINT_CASE(odrv->axis0.current_state, ENDPOINT_AXIS0_CURRENT_STATE); break;
        ENDPOINT_CASE(odrv->axis0.is_homed, ENDPOINT_AXIS0_IS_HOMED); break;
        ENDPOINT_CASE(odrv->axis0.min_endstop_config_enabled, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED); break;
        ENDPOINT_CASE(odrv->axis0.min_endstop_endstop_state, ENDPOINT_AXIS0_MIN_ENDSTOP_ENDSTOP_STATE); break;
        ENDPOINT_CASE(odrv->axis0.error, ENDPOINT_AXIS0_ERROR); break;
        ENDPOINT_CASE(odrv->axis0.motor_error, ENDPOINT_AXIS0_MOTOR_ERROR); break;
        ENDPOINT_CASE(odrv->axis0.encoder_error, ENDPOINT_AXIS0_ENCODER_ERROR); break;
        ENDPOINT_CASE(odrv->axis0.controller_error, ENDPOINT_AXIS0_CONTROLLER_ERROR); break;
        ENDPOINT_CASE(odrv->axis0.motor_is_calibrated, ENDPOINT_AXIS0_MOTOR_IS_CALIBRATED); break;
        ENDPOINT_CASE(odrv->axis0.motor_config_pre_calibrated, ENDPOINT_AXIS0_MOTOR_CONFIG_PRE_CALIBRATED); break;

        // Axis 1
        ENDPOINT_CASE(odrv->axis1.encoder_pos_estimate, ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE); break;
        ENDPOINT_CASE(odrv->axis1.current_state, ENDPOINT_AXIS1_CURRENT_STATE); break;
        ENDPOINT_CASE(odrv->axis1.is_homed, ENDPOINT_AXIS1_IS_HOMED); break;
        ENDPOINT_CASE(odrv->axis1.min_endstop_config_enabled, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED); break;
        ENDPOINT_CASE(odrv->axis1.min_endstop_endstop_state, ENDPOINT_AXIS1_MIN_ENDSTOP_ENDSTOP_STATE); break;
        ENDPOINT_CASE(odrv->axis1.error, ENDPOINT_AXIS1_ERROR); break;
        ENDPOINT_CASE(odrv->axis1.motor_error, ENDPOINT_AXIS1_MOTOR_ERROR); break;
        ENDPOINT_CASE(odrv->axis1.encoder_error, ENDPOINT_AXIS1_ENCODER_ERROR); break;
        ENDPOINT_CASE(odrv->axis1.controller_error, ENDPOINT_AXIS1_CONTROLLER_ERROR); break;
        ENDPOINT_CASE(odrv->axis1.motor_is_calibrated, ENDPOINT_AXIS1_MOTOR_IS_CALIBRATED); break;
        ENDPOINT_CASE(odrv->axis1.motor_config_pre_calibrated, ENDPOINT_AXIS1_MOTOR_CONFIG_PRE_CALIBRATED); break;

        ENDPOINT_DEFAULT_CASE();
    }
}

void OnDataCallback0(uint16_t endpointID, uint8_t* data, size_t dataSize) {
    OnDataCallback(&odrvData0, endpointID, data, dataSize);
}

void OnDataCallback1(uint16_t endpointID, uint8_t* data, size_t dataSize) {
    OnDataCallback(&odrvData1, endpointID, data, dataSize);
}

ODrive odrv0(Serial1, OnDataCallback0);
ODrive odrv1(Serial2, OnDataCallback1);

void setupODrives() {
    odrv0.begin(115200);
    odrv1.begin(115200);
}

void updateODrive(ODrive* odrv, ODriveProperties* prop) {

    // Clear any old data from the buffer
    odrv->flush();



    // Write every cycle
    WRITE(prop->axis0.controller_input_pos, ENDPOINT_AXIS0_CONTROLLER_INPUT_POS);
    WRITE(prop->axis1.controller_input_pos, ENDPOINT_AXIS1_CONTROLLER_INPUT_POS);
    
    // Read every cycle
    READ(ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE);
    READ(ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE);




    // Requests with triggers
    TRIGGER_RETURN(prop->axis0.requested_state, AXIS_STATE_UNDEFINED, ENDPOINT_AXIS0_REQUESTED_STATE);
    TRIGGER_RETURN(prop->axis0.controller_config_input_mode, INPUT_MODE_INACTIVE, ENDPOINT_AXIS0_CONTROLLER_CONFIG_INPUT_MODE);
    TRIGGER_RETURN(prop->axis1.requested_state, AXIS_STATE_UNDEFINED, ENDPOINT_AXIS1_REQUESTED_STATE);
    TRIGGER_RETURN(prop->axis1.controller_config_input_mode, INPUT_MODE_INACTIVE, ENDPOINT_AXIS1_CONTROLLER_CONFIG_INPUT_MODE);

    RUN_EVERY_NTH_CYCLE_START();

    // Now functions
    RUN_EVERY_NTH_CYCLE(FUNCTION(prop->axis0.clear_errors, ENDPOINT_AXIS0_CLEAR_ERRORS));
    RUN_EVERY_NTH_CYCLE(FUNCTION(prop->axis1.clear_errors, ENDPOINT_AXIS1_CLEAR_ERRORS));

    // Write every nth cycle
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.motor_config_calibration_current, ENDPOINT_AXIS0_MOTOR_CONFIG_CALIBRATION_CURRENT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.motor_config_current_lim, ENDPOINT_AXIS0_MOTOR_CONFIG_CURRENT_LIM));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.trap_traj_config_vel_limit, ENDPOINT_AXIS0_TRAP_TRAJ_CONFIG_VEL_LIMIT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.trap_traj_config_accel_limit, ENDPOINT_AXIS0_TRAP_TRAJ_CONFIG_ACCEL_LIMIT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.trap_traj_config_decel_limit, ENDPOINT_AXIS0_TRAP_TRAJ_CONFIG_DECEL_LIMIT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.controller_config_pos_gain, ENDPOINT_AXIS0_CONTROLLER_CONFIG_POS_GAIN));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.controller_config_vel_gain, ENDPOINT_AXIS0_CONTROLLER_CONFIG_VEL_GAIN));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.controller_config_vel_integrator_gain, ENDPOINT_AXIS0_CONTROLLER_CONFIG_VEL_INTEGRATOR_GAIN));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.min_endstop_config_enabled, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.min_endstop_config_gpio_num, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_GPIO_NUM));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.min_endstop_config_is_active_high, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_IS_ACTIVE_HIGH));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.min_endstop_config_offset, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_OFFSET));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.controller_config_homing_speed, ENDPOINT_AXIS0_CONTROLLER_CONFIG_HOMING_SPEED));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis0.controller_config_enable_overspeed_error, ENDPOINT_AXIS0_CONTROLLER_CONFIG_ENABLE_OVERSPEED_ERROR));

    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.motor_config_calibration_current, ENDPOINT_AXIS1_MOTOR_CONFIG_CALIBRATION_CURRENT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.motor_config_current_lim, ENDPOINT_AXIS1_MOTOR_CONFIG_CURRENT_LIM));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.trap_traj_config_vel_limit, ENDPOINT_AXIS1_TRAP_TRAJ_CONFIG_VEL_LIMIT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.trap_traj_config_accel_limit, ENDPOINT_AXIS1_TRAP_TRAJ_CONFIG_ACCEL_LIMIT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.trap_traj_config_decel_limit, ENDPOINT_AXIS1_TRAP_TRAJ_CONFIG_DECEL_LIMIT));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.controller_config_pos_gain, ENDPOINT_AXIS1_CONTROLLER_CONFIG_POS_GAIN));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.controller_config_vel_gain, ENDPOINT_AXIS1_CONTROLLER_CONFIG_VEL_GAIN));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.controller_config_vel_integrator_gain, ENDPOINT_AXIS1_CONTROLLER_CONFIG_VEL_INTEGRATOR_GAIN));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.min_endstop_config_enabled, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.min_endstop_config_gpio_num, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_GPIO_NUM));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.min_endstop_config_is_active_high, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_IS_ACTIVE_HIGH));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.min_endstop_config_offset, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_OFFSET));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.controller_config_homing_speed, ENDPOINT_AXIS1_CONTROLLER_CONFIG_HOMING_SPEED));
    RUN_EVERY_NTH_CYCLE(WRITE(prop->axis1.controller_config_enable_overspeed_error, ENDPOINT_AXIS1_CONTROLLER_CONFIG_ENABLE_OVERSPEED_ERROR));

    // Reads
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_VBUS_VOLTAGE));

    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_CURRENT_STATE));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_IS_HOMED));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_MIN_ENDSTOP_ENDSTOP_STATE));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_MOTOR_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_ENCODER_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_CONTROLLER_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_MOTOR_IS_CALIBRATED));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS0_MOTOR_CONFIG_PRE_CALIBRATED));

    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_CURRENT_STATE));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_IS_HOMED));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_MIN_ENDSTOP_ENDSTOP_STATE));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_MOTOR_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_ENCODER_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_CONTROLLER_ERROR));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_MOTOR_IS_CALIBRATED));
    RUN_EVERY_NTH_CYCLE(READ(ENDPOINT_AXIS1_MOTOR_CONFIG_PRE_CALIBRATED));

    RUN_EVERY_NTH_CYCLE_END();

}

void updateODrives() {

    uint32_t now = micros();
    if (now - lastUpdate >= cycleTime) {
        lastUpdate = now;
        
        updateODrive(&odrv0, &odrvData0);
        updateODrive(&odrv1, &odrvData1);
    }

    odrv0.update();
    odrv1.update();
}
