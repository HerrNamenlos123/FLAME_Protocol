
#include "Process.h"
#include "ODrive.h"
#include "Config.h"
#include "FLAME_Protocol.h"
#include "endpoints.h"

auto& rx = FLAME_Protocol::toMCU;
auto& tx = FLAME_Protocol::toPC;

uint32_t ignoreStart = 0;
uint32_t ignorePeriod = 0;
uint32_t calibrationStarted = 0;

ODrive odrv0(Serial1);
ODrive odrv1(Serial2);

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

template<uint16_t jsonCRC, uint16_t endpointID, typename T>
T __READ(ODrive& odrive, T _default) {
    T value;
    if (!odrive.read<jsonCRC, endpointID, T>(&value)) {
        return _default;
    }

    return value;
}

#define SET(odrive, endpoint, value) odrive.write<endpoint>(value, false)
#define READ(odrive, endpoint, default_val) __READ<endpoint>(odrive, default_val)

void loadCommonSettings(ODrive& odrv) {

    // End stops
    SET(odrv, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED, false);
    SET(odrv, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED, false);
    SET(odrv, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_IS_ACTIVE_HIGH, true);
    SET(odrv, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_IS_ACTIVE_HIGH, true);

    // Motors
    SET(odrv, ENDPOINT_AXIS0_MOTOR_CONFIG_CALIBRATION_CURRENT, MOTOR_CALIBRATION_CURRENT);
    SET(odrv, ENDPOINT_AXIS1_MOTOR_CONFIG_CALIBRATION_CURRENT, MOTOR_CALIBRATION_CURRENT);
    SET(odrv, ENDPOINT_AXIS0_MOTOR_CONFIG_CURRENT_LIM, MOTOR_DRIVE_CURRENT);
    SET(odrv, ENDPOINT_AXIS1_MOTOR_CONFIG_CURRENT_LIM, MOTOR_DRIVE_CURRENT);
    SET(odrv, ENDPOINT_AXIS0_MOTOR_CONFIG_POLE_PAIRS, MOTOR_POLE_PAIRS);
    SET(odrv, ENDPOINT_AXIS1_MOTOR_CONFIG_POLE_PAIRS, MOTOR_POLE_PAIRS);
    SET(odrv, ENDPOINT_AXIS0_MOTOR_CONFIG_TORQUE_CONSTANT, MOTOR_TORQUE_CONSTANT);
    SET(odrv, ENDPOINT_AXIS1_MOTOR_CONFIG_TORQUE_CONSTANT, MOTOR_TORQUE_CONSTANT);
    SET(odrv, ENDPOINT_AXIS0_CONTROLLER_CONFIG_ENABLE_OVERSPEED_ERROR, false);
    SET(odrv, ENDPOINT_AXIS1_CONTROLLER_CONFIG_ENABLE_OVERSPEED_ERROR, false);
    SET(odrv, ENDPOINT_AXIS0_CONFIG_CALIBRATION_LOCKIN_CURRENT, MOTOR_CALIBRATION_CURRENT);
    SET(odrv, ENDPOINT_AXIS1_CONFIG_CALIBRATION_LOCKIN_CURRENT, MOTOR_CALIBRATION_CURRENT);
    SET(odrv, ENDPOINT_AXIS0_CONFIG_CALIBRATION_LOCKIN_VEL, MOTOR_INDEX_SEARCH_LOCKIN_SPEED);
    SET(odrv, ENDPOINT_AXIS1_CONFIG_CALIBRATION_LOCKIN_VEL, -MOTOR_INDEX_SEARCH_LOCKIN_SPEED);
    
    SET(odrv, ENDPOINT_AXIS0_CONTROLLER_CONFIG_POS_GAIN, MOTOR_GAIN_P);
    SET(odrv, ENDPOINT_AXIS1_CONTROLLER_CONFIG_POS_GAIN, MOTOR_GAIN_P);
    SET(odrv, ENDPOINT_AXIS0_CONTROLLER_CONFIG_VEL_INTEGRATOR_GAIN, MOTOR_GAIN_I);
    SET(odrv, ENDPOINT_AXIS1_CONTROLLER_CONFIG_VEL_INTEGRATOR_GAIN, MOTOR_GAIN_I);
    SET(odrv, ENDPOINT_AXIS0_CONTROLLER_CONFIG_VEL_GAIN, MOTOR_GAIN_D);
    SET(odrv, ENDPOINT_AXIS1_CONTROLLER_CONFIG_VEL_GAIN, MOTOR_GAIN_D);

    // Encoder
    SET(odrv, ENDPOINT_AXIS0_ENCODER_CONFIG_CPR, ENCODER_CPR);
    SET(odrv, ENDPOINT_AXIS1_ENCODER_CONFIG_CPR, ENCODER_CPR);

}

void loadSpecificSettings() {

    SET(odrv0, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_GPIO_NUM, END_STOP_GPIO_AXIS0);
    SET(odrv0, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_GPIO_NUM, END_STOP_GPIO_AXIS1);
    SET(odrv1, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_GPIO_NUM, END_STOP_GPIO_AXIS2);
    SET(odrv1, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_GPIO_NUM, END_STOP_GPIO_AXIS3);

    SET(odrv0, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_OFFSET, END_STOP_OFFSET_AXIS0);
    SET(odrv0, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_OFFSET, END_STOP_OFFSET_AXIS1);
    SET(odrv1, ENDPOINT_AXIS0_MIN_ENDSTOP_CONFIG_OFFSET, END_STOP_OFFSET_AXIS2);
    SET(odrv1, ENDPOINT_AXIS1_MIN_ENDSTOP_CONFIG_OFFSET, END_STOP_OFFSET_AXIS3);

}

bool closedLoop() {     // If any of the 4 axes is in closed loop mode (or reading fails)
    int32_t state;
    if (READ(odrv0, ENDPOINT_AXIS0_CURRENT_STATE, AXIS_STATE_CLOSED_LOOP_CONTROL) == AXIS_STATE_CLOSED_LOOP_CONTROL) return true;
    if (READ(odrv0, ENDPOINT_AXIS1_CURRENT_STATE, AXIS_STATE_CLOSED_LOOP_CONTROL) == AXIS_STATE_CLOSED_LOOP_CONTROL) return true;
    //if (!READ(odrv1, ENDPOINT_AXIS0_CURRENT_STATE, AXIS_STATE_CLOSED_LOOP_CONTROL) == AXIS_STATE_CLOSED_LOOP_CONTROL) return true;
    //if (!READ(odrv1, ENDPOINT_AXIS1_CURRENT_STATE, AXIS_STATE_CLOSED_LOOP_CONTROL) == AXIS_STATE_CLOSED_LOOP_CONTROL) return true;
    return false;
}

bool precalibrated() {      // If all motors and encoders are precalibrated and all readings are valid
    if (!READ(odrv0, ENDPOINT_AXIS0_MOTOR_CONFIG_PRE_CALIBRATED, false)) return false;
    //if (!READ(odrv0, ENDPOINT_AXIS1_MOTOR_CONFIG_PRE_CALIBRATED, false)) return false;
    //if (!READ(odrv1, ENDPOINT_AXIS0_MOTOR_CONFIG_PRE_CALIBRATED, false)) return false;
    //if (!READ(odrv1, ENDPOINT_AXIS1_MOTOR_CONFIG_PRE_CALIBRATED, false)) return false;
    if (!READ(odrv0, ENDPOINT_AXIS0_ENCODER_CONFIG_PRE_CALIBRATED, false)) return false;
    //if (!READ(odrv0, ENDPOINT_AXIS1_ENCODER_CONFIG_PRE_CALIBRATED, false)) return false;
    //if (!READ(odrv1, ENDPOINT_AXIS0_ENCODER_CONFIG_PRE_CALIBRATED, false)) return false;
    //if (!READ(odrv1, ENDPOINT_AXIS1_ENCODER_CONFIG_PRE_CALIBRATED, false)) return false;
    return true;
}

bool anyErrors() {
    uint32_t error = 0;
    error |= READ(odrv0, ENDPOINT_AXIS0_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS0_MOTOR_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS0_ENCODER_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS0_CONTROLLER_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS1_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS1_MOTOR_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS1_ENCODER_ERROR, 0);
    error |= READ(odrv0, ENDPOINT_AXIS1_CONTROLLER_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS0_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS0_MOTOR_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS0_ENCODER_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS0_CONTROLLER_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS1_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS1_MOTOR_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS1_ENCODER_ERROR, 0);
    //error |= READ(odrv1, ENDPOINT_AXIS1_CONTROLLER_ERROR, 0);
    return error != 0;
}

void clearErrors() {
    odrv0.write<ENDPOINT_AXIS0_CLEAR_ERRORS>(true);
    odrv0.write<ENDPOINT_AXIS1_CLEAR_ERRORS>(true);
}

template<uint16_t endpointID_req, uint16_t endpointID_cur, uint16_t endpointID_endstop>
bool home(ODrive& odrv, const char* str) {

    Serial.print("Finding index on ");
    Serial.println(str);
    odrv.write<JSON_CRC, endpointID_req, int32_t>(AXIS_STATE_ENCODER_INDEX_SEARCH);
    while (__READ<JSON_CRC, endpointID_cur, int32_t>(odrv, AXIS_STATE_UNDEFINED) != AXIS_STATE_IDLE);
    odrv.write<JSON_CRC, endpointID_req, int32_t>(AXIS_STATE_CLOSED_LOOP_CONTROL);
    if (anyErrors()) { Serial.println("Failed"); return false; }
    Serial.println("done");

    Serial.print("Homing ");
    Serial.println(str);
    odrv.write<JSON_CRC, endpointID_endstop, bool>(true);
    delay(50);
    odrv.write<JSON_CRC, endpointID_req, int32_t>(AXIS_STATE_HOMING);
    while (__READ<JSON_CRC, endpointID_cur, int32_t>(odrv, AXIS_STATE_UNDEFINED) != AXIS_STATE_IDLE);
    odrv.write<JSON_CRC, endpointID_req, int32_t>(AXIS_STATE_CLOSED_LOOP_CONTROL);
    if (anyErrors()) { Serial.println("Failed"); return false; }
    Serial.println("done");

    return true;
}

bool home() {

    if (!home<ENDPOINT_ID_AXIS0_REQUESTED_STATE, ENDPOINT_ID_AXIS0_CURRENT_STATE, ENDPOINT_ID_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED>(
        odrv0, "odrv0.axis0")) return false;
    if (!home<ENDPOINT_ID_AXIS1_REQUESTED_STATE, ENDPOINT_ID_AXIS1_CURRENT_STATE, ENDPOINT_ID_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED>(
        odrv0, "odrv0.axis1")) return false;
    //if (!home<ENDPOINT_ID_AXIS0_REQUESTED_STATE, ENDPOINT_ID_AXIS0_CURRENT_STATE, ENDPOINT_ID_AXIS0_MIN_ENDSTOP_CONFIG_ENABLED>(
    //    odrv1, "odrv1.axis0")) return false;
    //if (!home<ENDPOINT_ID_AXIS1_REQUESTED_STATE, ENDPOINT_ID_AXIS1_CURRENT_STATE, ENDPOINT_ID_AXIS1_MIN_ENDSTOP_CONFIG_ENABLED>(
    //    odrv1, "odrv1.axis1")) return false;
    
    return true;
}

void initialize() {

    Serial.println("Initializing odrives");

    odrv0.begin(ODRIVE_BAUD_RATE);
    odrv1.begin(ODRIVE_BAUD_RATE);

    // First check if both odrives are available
    while (true) {
        float v;
        if (odrv0.read<ENDPOINT_VBUS_VOLTAGE>(&v)) {
            break;
        }
        Serial.println("ODrive 0 does not respond!");
        delay(1000);
    }
    //if (!odrv1.read<ENDPOINT_VBUS_VOLTAGE>(&v)) {
    //    Serial.println("ODrive 1 does not respond!");
    //    while (true);
    //}

    // Check if it is already running
    if (closedLoop()) {
        Serial.println("At least one axis might be in closed loop mode. Please shut down and restart!");
        Serial.println("Do you want to restart both odrives now? [Y/N]");

        while (true) {
            if (Serial.available()) {
                char c = Serial.read();
                if (c == 'Y' || c == 'y') {
                    break;
                }
            }
        }

        Serial.println("Warning, robot will collapse now! Restarting both odrives in 5 seconds...");
        delay(5000);
        Serial.println("Rebooting now...");
        odrv0.write<ENDPOINT_REBOOT>(true);
        odrv1.write<ENDPOINT_REBOOT>(true);
        Serial.println("Reboot requested");
        initialize();
        return;
    }

    // Check if the motors are precalibrated
    if (!precalibrated()) {
        Serial.println("At least one motor is not precalibrated! Please precalibrate manually!");
        Serial.println("First do full calibration, then index search, then offset calibration, "
            "then set precalibrated to true and save.");
        while (true);
    }

    // Load all settings
    loadCommonSettings(odrv0);
    loadCommonSettings(odrv1);
    loadSpecificSettings();

    // Clear errors
    clearErrors();

    // Find index and home each axis
    if (!home()) {
        Serial.println("Homing/Index search failed");
        while (true);
    }
    
}

void update() {

    float axis0 = max(min(rx.desiredAxis1, AXIS0_MAX_ANGLE), AXIS0_MIN_ANGLE);
    float axis1 = max(min(rx.desiredAxis2, AXIS1_MAX_ANGLE), AXIS1_MIN_ANGLE);
    float axis2 = max(min(rx.desiredAxis3, AXIS2_MAX_ANGLE), AXIS2_MIN_ANGLE);
    float axis3 = max(min(rx.desiredAxis4, AXIS3_MAX_ANGLE), AXIS3_MIN_ANGLE);

    axis0 = (axis0 - AXIS0_ZERO_OFFSET) / 360.f * AXIS0_REDUCTION_RATIO;
    axis1 = -(axis1 - AXIS1_ZERO_OFFSET) / 360.f * AXIS1_REDUCTION_RATIO;
    axis2 = (axis2 - AXIS2_ZERO_OFFSET) / 360.f * AXIS2_REDUCTION_RATIO;
    axis3 = (axis3 - AXIS3_ZERO_OFFSET) / 360.f * AXIS3_REDUCTION_RATIO;


    uint16_t seq_ax0 = odrv0.sendReadRequest<ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE>();
    uint16_t seq_ax1 = odrv0.sendReadRequest<ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE>();
    //uint16_t seq_ax2 = odrv1.sendReadRequest<ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE>();
    //uint16_t seq_ax3 = odrv1.sendReadRequest<ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE>();

    if (!tx.safetyMode) {
        odrv0.write<ENDPOINT_AXIS0_CONTROLLER_INPUT_POS>(axis0, false);
        odrv0.write<ENDPOINT_AXIS1_CONTROLLER_INPUT_POS>(axis1, false);
        //odrv1.write<ENDPOINT_AXIS0_CONTROLLER_INPUT_POS>(axis2, false);
        //odrv1.write<ENDPOINT_AXIS1_CONTROLLER_INPUT_POS>(axis3, false);
    }

    odrv0.waitForResponse<ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE>(&tx.actualAxis1, seq_ax0);
    odrv0.waitForResponse<ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE>(&tx.actualAxis2, seq_ax1);
    //odrv1.waitForResponse<ENDPOINT_AXIS0_ENCODER_POS_ESTIMATE>(&tx.actualAxis3, seq_ax2);
    //odrv1.waitForResponse<ENDPOINT_AXIS1_ENCODER_POS_ESTIMATE>(&tx.actualAxis4, seq_ax3);

    if (tx.safetyMode) {
        rx.desiredAxis1 = tx.actualAxis1;
        rx.desiredAxis2 = tx.actualAxis2;
        rx.desiredAxis3 = tx.actualAxis3;
        rx.desiredAxis4 = tx.actualAxis4;
    }

}




/*

void checkSafetyMode() {
}

void startProcess() {
    ignoreFor(2000);    // Let the odrives run for 2 seconds before doing anything

    odrv0->axis0.clear_errors++;
    odrv0->axis1.clear_errors++;
    odrv1->axis0.clear_errors++;
    odrv1->axis1.clear_errors++;

    odrv0->axis0.controller_config_homing_speed = HOMING_SPEED;
    odrv0->axis1.controller_config_homing_speed = -HOMING_SPEED;
    odrv1->axis0.controller_config_homing_speed = HOMING_SPEED;
    odrv1->axis1.controller_config_homing_speed = HOMING_SPEED;

    setupMotor(&odrv0->axis0, CALIBRATION_CURRENT, DRIVE_CURRENT, MOTOR_TORQUE_CONSTANT);
    setupMotor(&odrv0->axis1, CALIBRATION_CURRENT, DRIVE_CURRENT, MOTOR_TORQUE_CONSTANT);
    setupMotor(&odrv1->axis0, CALIBRATION_CURRENT, DRIVE_CURRENT, MOTOR_TORQUE_CONSTANT);
    setupMotor(&odrv1->axis1, CALIBRATION_CURRENT, DRIVE_CURRENT, MOTOR_TORQUE_CONSTANT);

    setPID(&odrv0->axis0, GAIN_P, GAIN_I, GAIN_D);
    setPID(&odrv0->axis1, GAIN_P, GAIN_I, GAIN_D);
    setPID(&odrv1->axis0, GAIN_P, GAIN_I, GAIN_D);
    setPID(&odrv1->axis1, GAIN_P, GAIN_I, GAIN_D);

}

void updateProcess() {

    if (state == State::DISCONNECTED) {
        setInputPos(&odrv0->axis0, odrv0->axis0.encoder_pos_estimate);
        setInputPos(&odrv0->axis1, odrv0->axis1.encoder_pos_estimate);
        setInputPos(&odrv1->axis0, odrv1->axis0.encoder_pos_estimate);
        setInputPos(&odrv1->axis1, odrv1->axis1.encoder_pos_estimate);
    }

    if (millis() - ignoreStart < ignorePeriod) {
        return;
    }

    if (updateState()) {
        return;
    }

    checkSettings(odrv0, "odrv0");
    //checkSettings(odrv1, "odrv1");

    //if (state == State::IDLE_UNHOMED) {
    //    checkClosedLoop(odrv0, &tx.odrive0Error);
    //    checkClosedLoop(odrv1, &tx.odrive1Error);
    //}

    checkErrors();

    //checkSafetyMode();
}*/
