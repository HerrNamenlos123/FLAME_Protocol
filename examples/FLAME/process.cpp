
#include "Process.h"
#include "ODrive.h"
#include "Config.h"
#include "FLAME_Protocol.h"
#include "ODriveBackend.h"

#define ENDSTOP_OFFSET_AXIS0 -0.5
#define ENDSTOP_OFFSET_AXIS1 0.5
#define ENDSTOP_OFFSET_AXIS2 0
#define ENDSTOP_OFFSET_AXIS3 0

#define CALIBRATION_CURRENT 30.f
#define DRIVE_CURRENT 50.f
#define MOTOR_TORQUE_CONSTANT (8.27f / 150.f)

#define TR_HOMING_ACCEL 2.f
#define TR_HOMING_VEL 2.f
#define TR_HOMING_DECEL 2.f
#define TR_HOMING_POS_OVER_ENDSTOP 0.5f    // Rotations
#define HOMING_SPEED 0.05

#define GAIN_P 130.f
#define GAIN_I 0.5f
#define GAIN_D 0.2f

auto& rx = FLAME_Protocol::toMCU;
auto& tx = FLAME_Protocol::toPC;
static auto odrv0 = &odrvData0;
static auto odrv1 = &odrvData1;

uint32_t ignoreStart = 0;
uint32_t ignorePeriod = 0;
uint32_t calibrationStarted = 0;

enum class State {
    DISCONNECTED,
    CLEAR_ENDSTOPS,
    CLEAR_ERRORS,
    ERROR,
    ERROR_UNRECOVERABLE,
    UNCONFIGURED,
    IDLE_UNHOMED,
    MOVE_OVER_ENDSTOP,
    WAIT_CALIBRATION_START,
    CALIBRATION,
    CALIBRATION_DONE,
    ENABLE_ENDSTOPS,
    START_HOMING,
    HOMING,
    IDLE,
    RUN,
    ERROR_WAIT_FOREVER,
    CLOSED_LOOP
};

enum State state = State::DISCONNECTED;

void ignoreFor(uint32_t milliseconds) {
    ignoreStart = millis();
    ignorePeriod = milliseconds;
}

void checkClosedLoop(ODriveProperties* odrv, bool* errorVar) {
    if (odrv->axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        if (tx.safetyMode) 
            odrv->axis0.controller_config_input_mode = INPUT_MODE_TRAP_TRAJ;
        else 
            odrv->axis0.controller_config_input_mode = INPUT_MODE_POS_FILTER;
    }
    else {
        *errorVar = true;
    }
    if (odrv->axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        if (tx.safetyMode) 
            odrv->axis1.controller_config_input_mode = INPUT_MODE_TRAP_TRAJ;
        else 
            odrv->axis1.controller_config_input_mode = INPUT_MODE_POS_FILTER;
    }
    else {
        *errorVar = true;
    }
}

void checkErrors() {
    tx.odrive0Axis0Error = odrv0->axis0.error || odrv0->axis0.motor_error || odrv0->axis0.encoder_error || odrv0->axis0.controller_error;
    tx.odrive0Axis1Error = odrv0->axis1.error || odrv0->axis1.motor_error || odrv0->axis1.encoder_error || odrv0->axis1.controller_error;
    tx.odrive0Error = tx.odrive0Axis0Error || tx.odrive0Axis1Error;

    tx.odrive1Axis0Error = odrv1->axis0.error || odrv1->axis0.motor_error || odrv1->axis0.encoder_error || odrv1->axis0.controller_error;
    tx.odrive1Axis1Error = odrv1->axis1.error || odrv1->axis1.motor_error || odrv1->axis1.encoder_error || odrv1->axis1.controller_error;
    tx.odrive1Error = tx.odrive1Axis0Error || tx.odrive1Axis1Error;

    if ((tx.odrive0Error || tx.odrive1Error) && 
        state != State::ERROR_UNRECOVERABLE && 
        state != State::ERROR_WAIT_FOREVER && 
        state != State::CLEAR_ENDSTOPS && 
        state != State::CLEAR_ERRORS && 
        state != State::DISCONNECTED) 
    {
        state = State::ERROR;
    }
}

void checkSettings(ODriveProperties* odrv, const char* odrvString) {

    if (state == State::ERROR || state == State::ERROR_UNRECOVERABLE || state == State::ERROR_WAIT_FOREVER)
        return;

    if (!odrv->axis0.motor_config_pre_calibrated) {
        Serial.print(odrvString);
        Serial.println(".axis0's motor is not precalibrated! Please calibrate manually");
        state = State::ERROR_UNRECOVERABLE;
    }
    if (!odrv->axis1.motor_config_pre_calibrated) {
        Serial.print(odrvString);
        Serial.println(".axis1's motor is not precalibrated! Please calibrate manually");
        state = State::ERROR_UNRECOVERABLE;
    }
}

void setTrapTraj(Axis* axis, float accel, float vel, float decel) {
    axis->controller_config_input_mode = INPUT_MODE_TRAP_TRAJ;
    axis->trap_traj_config_accel_limit = accel;
    axis->trap_traj_config_vel_limit = vel;
    axis->trap_traj_config_decel_limit = decel;
}

void setInputPos(Axis* axis, float inputPos) {
    axis->controller_input_pos = inputPos;
}

void setPID(Axis* axis, float p, float i, float d) {
    axis->controller_config_pos_gain = p;
    axis->controller_config_vel_integrator_gain = i;
    axis->controller_config_vel_gain = d;
}

void setupEndstop(Axis* axis, bool enabled, uint16_t gpio, bool activeHigh, float offset) {
    axis->min_endstop_config_enabled = enabled;
    axis->min_endstop_config_gpio_num = gpio;
    axis->min_endstop_config_is_active_high = activeHigh;
    axis->min_endstop_config_offset = offset;
}

bool updateState() {

    bool returnNow = false;

    bool armed = false;
    switch (state) {
        case State::ERROR:
            Serial.println("Error detected: ");
            Serial.println(odrv0->axis0.error);
            Serial.println(odrv0->axis0.motor_error);
            Serial.println(odrv0->axis0.encoder_error);
            Serial.println(odrv0->axis0.controller_error);
            Serial.println(odrv0->axis1.error);
            Serial.println(odrv0->axis1.motor_error);
            Serial.println(odrv0->axis1.encoder_error);
            Serial.println(odrv0->axis1.controller_error);
            //state = State::DISCONNECTED;
            //odrv0->axis0.clear_errors++;
            //odrv0->axis1.clear_errors++;
            ignoreFor(2000);
            returnNow = true;
            break;

        case State::ERROR_UNRECOVERABLE:
            Serial.println("Unrecoverable error detected, stopping machine");
            state = State::ERROR_WAIT_FOREVER;
            returnNow = true;
            break;

        case State::DISCONNECTED:

            if (odrv0->axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) armed = true;
            if (odrv0->axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) armed = true;
            if (odrv1->axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) armed = true;
            if (odrv1->axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) armed = true;
            if (armed) {
                Serial.println("Already armed! Not implemented, please unplug and restart machine!");
                state = State::ERROR_UNRECOVERABLE;
                returnNow = true;
                while (true);
                return;
            }

            setupEndstop(&odrv0->axis0, false, 3, true, ENDSTOP_OFFSET_AXIS0);
            setupEndstop(&odrv0->axis1, false, 4, true, ENDSTOP_OFFSET_AXIS1);
            setupEndstop(&odrv1->axis0, false, 3, true, ENDSTOP_OFFSET_AXIS2);
            setupEndstop(&odrv1->axis1, false, 4, true, ENDSTOP_OFFSET_AXIS3);
            state = State::CLEAR_ENDSTOPS;
            Serial.println("Disabled endstops");
            ignoreFor(800);
            break;

        case State::CLEAR_ENDSTOPS:
            odrv0->axis0.clear_errors++;
            odrv0->axis1.clear_errors++;
            odrv1->axis0.clear_errors++;
            odrv1->axis1.clear_errors++;
            state = State::CLEAR_ERRORS;
            Serial.println("Cleared errors");
            ignoreFor(800);
            break;

        case State::CLEAR_ERRORS:
            odrv0->axis0.requested_state = AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION;
            odrv0->axis1.requested_state = AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION;
            odrv1->axis0.requested_state = AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION;
            odrv1->axis1.requested_state = AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION;
            state = State::WAIT_CALIBRATION_START;
            calibrationStarted = millis();
            Serial.println("Waiting for odrives to approve calibration");
            break;

        case State::WAIT_CALIBRATION_START:
            if (odrv0->axis0.current_state == AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION && 
                odrv0->axis1.current_state == AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION) {
                state = State::CALIBRATION;
                Serial.println("Calibrating has started");
            }
            if (millis() - calibrationStarted > 10000) {
                Serial.println("Timeout, odrives will not start calibrating");
                odrv0->axis0.requested_state = AXIS_STATE_IDLE;
                odrv0->axis1.requested_state = AXIS_STATE_IDLE;
                odrv1->axis0.requested_state = AXIS_STATE_IDLE;
                odrv1->axis1.requested_state = AXIS_STATE_IDLE;
                state = State::ERROR_UNRECOVERABLE;
                returnNow = true;
            }
            break;

        case State::CALIBRATION:
            if (odrv0->axis0.current_state == AXIS_STATE_IDLE && odrv0->axis1.current_state == AXIS_STATE_IDLE) {
                Serial.println("Finished calibrating motor");
                state = State::CALIBRATION_DONE;
                ignoreFor(2000);
                returnNow = true;
            }
            break;

        case State::CALIBRATION_DONE:
            setTrapTraj(&odrv0->axis0, TR_HOMING_ACCEL, TR_HOMING_VEL, TR_HOMING_DECEL);
            setTrapTraj(&odrv0->axis1, TR_HOMING_ACCEL, TR_HOMING_VEL, TR_HOMING_DECEL);
            setTrapTraj(&odrv1->axis0, TR_HOMING_ACCEL, TR_HOMING_VEL, TR_HOMING_DECEL);
            setTrapTraj(&odrv1->axis1, TR_HOMING_ACCEL, TR_HOMING_VEL, TR_HOMING_DECEL);
            setInputPos(&odrv0->axis0, 0);
            setInputPos(&odrv0->axis1, 0);
            setInputPos(&odrv1->axis0, 0);
            setInputPos(&odrv1->axis1, 0);
            Serial.println("Enabled trap traj");
            state = State::IDLE_UNHOMED;
            ignoreFor(2000);
            returnNow = true;
            break;

        case State::IDLE_UNHOMED:
            odrv0->axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            odrv0->axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            odrv1->axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            odrv1->axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial.println("Enabled closed loop control");
            state = State::MOVE_OVER_ENDSTOP;
            ignoreFor(2000);
            break;

        case State::MOVE_OVER_ENDSTOP:
            setInputPos(&odrv0->axis0, TR_HOMING_POS_OVER_ENDSTOP);
            setInputPos(&odrv0->axis1, -TR_HOMING_POS_OVER_ENDSTOP);
            setInputPos(&odrv1->axis0, TR_HOMING_POS_OVER_ENDSTOP);
            setInputPos(&odrv1->axis1, TR_HOMING_POS_OVER_ENDSTOP);
            Serial.println("Moving over endstop");
            state = State::ENABLE_ENDSTOPS;
            ignoreFor(4000);
            break;

        case State::ENABLE_ENDSTOPS:
            setupEndstop(&odrv0->axis0, true, 3, true, ENDSTOP_OFFSET_AXIS0);
            setupEndstop(&odrv0->axis1, true, 4, true, ENDSTOP_OFFSET_AXIS1);
            setupEndstop(&odrv1->axis0, true, 3, true, ENDSTOP_OFFSET_AXIS2);
            setupEndstop(&odrv1->axis1, true, 4, true, ENDSTOP_OFFSET_AXIS3);
            Serial.println("Enabled endstops");
            state = State::START_HOMING;
            ignoreFor(500);
            break;

        case State::START_HOMING:
            odrv0->axis0.requested_state = AXIS_STATE_HOMING;
            odrv0->axis1.requested_state = AXIS_STATE_HOMING;
            odrv1->axis0.requested_state = AXIS_STATE_HOMING;
            odrv1->axis1.requested_state = AXIS_STATE_HOMING;
            setInputPos(&odrv0->axis0, 0);
            setInputPos(&odrv0->axis1, 0);
            setInputPos(&odrv1->axis0, 0);
            setInputPos(&odrv1->axis1, 0);
            Serial.println("Homing");
            state = State::HOMING;
            break;

        case State::HOMING:
            if (odrv0->axis0.current_state == AXIS_STATE_IDLE) odrv0->axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            if (odrv0->axis1.current_state == AXIS_STATE_IDLE) odrv0->axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            if (odrv1->axis0.current_state == AXIS_STATE_IDLE) odrv1->axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            if (odrv1->axis1.current_state == AXIS_STATE_IDLE) odrv1->axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            
            if (odrv0->axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL && 
                odrv0->axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL && 
                odrv1->axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL && 
                odrv1->axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
                    state = State::RUN;
                    Serial.println("Successfully inited, entering run mode");
                }
            break;

        case State::RUN:
            break;

        case State::ERROR_WAIT_FOREVER:       // Do nothing after error
            break;
    }

    return returnNow;
}

void checkSafetyMode() {
    if (!tx.safetyMode) {
        odrv0->axis0.controller_input_pos = max(min(rx.desiredAxis1, AXIS0_MAX_ANGLE), AXIS0_MIN_ANGLE);
        odrv0->axis1.controller_input_pos = -max(min(rx.desiredAxis2, AXIS1_MAX_ANGLE), AXIS1_MIN_ANGLE);
        odrv1->axis0.controller_input_pos = max(min(rx.desiredAxis3, AXIS2_MAX_ANGLE), AXIS2_MIN_ANGLE);
        odrv1->axis1.controller_input_pos = max(min(rx.desiredAxis4, AXIS3_MAX_ANGLE), AXIS3_MIN_ANGLE);
        tx.actualAxis1 = odrv0->axis0.encoder_pos_estimate;
        tx.actualAxis2 = -odrv0->axis1.encoder_pos_estimate;
        tx.actualAxis3 = odrv1->axis0.encoder_pos_estimate;
        tx.actualAxis4 = odrv1->axis1.encoder_pos_estimate;
    }
    else {      // Safety mode
        rx.desiredAxis1 = odrv0->axis0.controller_input_pos;
        rx.desiredAxis2 = -odrv0->axis1.controller_input_pos;
        rx.desiredAxis3 = odrv1->axis0.controller_input_pos;
        rx.desiredAxis4 = odrv1->axis1.controller_input_pos;
    }
}

void setupMotor(Axis* axis, float calibCurrent, float driveCurrent, float torqueConstant) {
    axis->motor_config_calibration_current = calibCurrent;
    axis->motor_config_current_lim = driveCurrent;
    axis->motor_config_torque_constant = torqueConstant;
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
}
