
#include "ODriveBackend.h"
#include "ODriveProperties.h"
#include "endpoints.h"

ODriveProperties odrvData0;
ODriveProperties odrvData1;
uint32_t lastUpdate = 0;
uint32_t cycleTime = 100000;

void OnDataCallback(ODriveProperties* odrv, uint16_t endpointID, uint8_t* data, size_t dataSize) {

    switch (endpointID) {
        ENDPOINT_CASE(odrv->axis0_encoder_pos_estimate, ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE); break;
        ENDPOINT_CASE(odrv->vbus_voltage, ENDPOINT_VBUS_VOLTAGE); break;
        ENDPOINT_CASE(odrv->axis0_current_state, ENDPOINT_AXIS0_CURRENT_STATE); break;
        ENDPOINT_CASE(odrv->axis0_requested_state, ENDPOINT_AXIS0_REQUESTED_STATE); break;
        ENDPOINT_CASE(odrv->axis0_is_homed, ENDPOINT_AXIS0_IS_HOMED); break;
        ENDPOINT_CASE(odrv->axis0_min_endstop_config_enabled, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED); break;
        ENDPOINT_CASE(odrv->axis0_min_endstop_endstop_state, ENDPOINT_AXIS0_MIN_ENDSTOP_ENDSTOP_STATE); break;
        ENDPOINT_CASE(odrv->axis0_error, ENDPOINT_AXIS0_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_motor_error, ENDPOINT_AXIS0_MOTOR_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_encoder_error, ENDPOINT_AXIS0_ENCODER_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_controller_error, ENDPOINT_AXIS0_CONTROLLER_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_motor_is_calibrated, ENDPOINT_AXIS0_MOTOR_IS_CALIBRATED); break;
        ENDPOINT_CASE(odrv->axis0_encoder_pos_estimate, ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE); break;
        ENDPOINT_CASE(odrv->axis0_current_state, ENDPOINT_AXIS1_CURRENT_STATE); break;
        ENDPOINT_CASE(odrv->axis0_requested_state, ENDPOINT_AXIS1_REQUESTED_STATE); break;
        ENDPOINT_CASE(odrv->axis0_is_homed, ENDPOINT_AXIS1_IS_HOMED); break;
        ENDPOINT_CASE(odrv->axis0_min_endstop_config_enabled, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED); break;
        ENDPOINT_CASE(odrv->axis0_min_endstop_endstop_state, ENDPOINT_AXIS1_MIN_ENDSTOP_ENDSTOP_STATE); break;
        ENDPOINT_CASE(odrv->axis0_error, ENDPOINT_AXIS1_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_motor_error, ENDPOINT_AXIS1_MOTOR_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_encoder_error, ENDPOINT_AXIS1_ENCODER_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_controller_error, ENDPOINT_AXIS1_CONTROLLER_ERROR); break;
        ENDPOINT_CASE(odrv->axis0_motor_is_calibrated, ENDPOINT_AXIS1_MOTOR_IS_CALIBRATED); break;
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

    // Write requests
    odrv->sendWriteRequest<ENDPOINT_AXIS0_CONTROLLER_INPUT_POS>(prop->axis0_controller_input_pos);

    // Requests with triggers
    if (prop->axis0_requested_state != AXIS_STATE_UNDEFINED) {
        odrv->sendWriteRequest<ENDPOINT_AXIS0_REQUESTED_STATE>(prop->axis0_requested_state);
        prop->axis0_requested_state = AXIS_STATE_UNDEFINED;
    }
    if (prop->axis0_controller_config_input_mode != INPUT_MODE_INACTIVE) {
        odrv->sendWriteRequest<ENDPOINT_AXIS0_CONTROLLER_CONFIG_INPUT_MODE>(prop->axis0_controller_config_input_mode);
        prop->axis0_controller_config_input_mode = INPUT_MODE_INACTIVE;
    }
    if (prop->axis1_requested_state != AXIS_STATE_UNDEFINED) {
        odrv->sendWriteRequest<ENDPOINT_AXIS1_REQUESTED_STATE>(prop->axis1_requested_state);
        prop->axis1_requested_state = AXIS_STATE_UNDEFINED;
    }
    if (prop->axis1_controller_config_input_mode != INPUT_MODE_INACTIVE) {
        odrv->sendWriteRequest<ENDPOINT_AXIS1_CONTROLLER_CONFIG_INPUT_MODE>(prop->axis1_controller_config_input_mode);
        prop->axis1_controller_config_input_mode = INPUT_MODE_INACTIVE;
    }

    // Now functions
    while (prop->axis0_clear_errors) {
        odrv->sendWriteRequest<ENDPOINT_AXIS0_CLEAR_ERRORS>(true);
        prop->axis0_clear_errors--;
    }
    while (prop->axis1_clear_errors) {
        odrv->sendWriteRequest<ENDPOINT_AXIS1_CLEAR_ERRORS>(true);
        prop->axis1_clear_errors--;
    }
    
    // Read requests
    odrv->sendReadRequest<ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE>();
    odrv->sendReadRequest<ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE>();

    switch (prop->propertyIndex) {
    case  0: odrv->sendReadRequest<ENDPOINT_VBUS_VOLTAGE>(); break;
    case  1: odrv->sendReadRequest<ENDPOINT_AXIS0_CURRENT_STATE>(); break;
    case  2: odrv->sendReadRequest<ENDPOINT_AXIS0_REQUESTED_STATE>(); break;
    case  3: odrv->sendReadRequest<ENDPOINT_AXIS0_IS_HOMED>(); break;
    case  4: odrv->sendReadRequest<ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED>(); break;
    case  5: odrv->sendReadRequest<ENDPOINT_AXIS0_MIN_ENDSTOP_ENDSTOP_STATE>(); break;
    case  6: odrv->sendReadRequest<ENDPOINT_AXIS0_ERROR>(); break;
    case  7: odrv->sendReadRequest<ENDPOINT_AXIS0_MOTOR_ERROR>(); break;
    case  8: odrv->sendReadRequest<ENDPOINT_AXIS0_ENCODER_ERROR>(); break;
    case  9: odrv->sendReadRequest<ENDPOINT_AXIS0_CONTROLLER_ERROR>(); break;
    case 10: odrv->sendReadRequest<ENDPOINT_AXIS0_MOTOR_IS_CALIBRATED>(); break;
    case 11: odrv->sendReadRequest<ENDPOINT_AXIS1_CURRENT_STATE>(); break;
    case 12: odrv->sendReadRequest<ENDPOINT_AXIS1_REQUESTED_STATE>(); break;
    case 13: odrv->sendReadRequest<ENDPOINT_AXIS1_IS_HOMED>(); break;
    case 14: odrv->sendReadRequest<ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED>(); break;
    case 15: odrv->sendReadRequest<ENDPOINT_AXIS1_MIN_ENDSTOP_ENDSTOP_STATE>(); break;
    case 16: odrv->sendReadRequest<ENDPOINT_AXIS1_ERROR>(); break;
    case 17: odrv->sendReadRequest<ENDPOINT_AXIS1_MOTOR_ERROR>(); break;
    case 18: odrv->sendReadRequest<ENDPOINT_AXIS1_ENCODER_ERROR>(); break;
    case 19: odrv->sendReadRequest<ENDPOINT_AXIS1_CONTROLLER_ERROR>(); break;
    case 20: odrv->sendReadRequest<ENDPOINT_AXIS1_MOTOR_IS_CALIBRATED>(); break;
    default: prop->propertyIndex = 0;   break;      // Nothing is sent the last time, who cares
    }
    prop->propertyIndex++;

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
