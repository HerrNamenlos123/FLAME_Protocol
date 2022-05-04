
#include "Process.h"
#include "ODrive.h"
#include "Config.h"
#include "FLAME_Protocol.h"
#include "ODriveBackend.h"

#define MAX_RESPONSE_TIMEOUT 500
#define ERROR_CLEAR_DELAY 500
#define HOMING_DELAY 500

uint32_t processCount = 0;
uint32_t ignoreStart = 0;
uint32_t ignorePeriod = 0;

enum class State {
    DISCONNECTED,
    ERROR,
    UNCONFIGURED,
    IDLE_UNHOMED,
    WAIT_CALIBRATION_START,
    CALIBRATION,
    HOMING,
    IDLE,
    CLOSED_LOOP
};

enum State state = State::DISCONNECTED;

/*
bool checkError(ODrive* odrv, State state) {
    if (odrv->axis0_error || odrv->axis0_motor_error || odrv->axis0_encoder_error || odrv->axis0_controller_error) {

        if (odrv->axis0_error == 0x1000 && 
            odrv->axis0_motor_error == 0 && 
            odrv->axis0_encoder_error == 0 && 
            odrv->axis0_controller_error == 0 && 
            odrv->state != State::CLOSED_LOOP) 
        {
            // Ignore the error
            return false;
        }

        odrv->state = state;

        odrv->delayStartTime = millis();
        odrv->delayTime = ERROR_CLEAR_DELAY;
        odrv->delayNeeded = true;

        Serial.print("Axis 0 error: ");
        Serial.println(odrv->axis0_error);
        Serial.print("Motor 0 error: ");
        Serial.println(odrv->axis0_motor_error);
        Serial.print("Encoder 0 error: ");
        Serial.println(odrv->axis0_encoder_error);
        Serial.print("Controller 0 error: ");
        Serial.println(odrv->axis0_controller_error);

        if (odrv->axis0_error & 0x0001) {
            Serial.println("Invalid state detected! Switching to State::IDLE_UNHOMED!");
            odrv->axis0_clear_errors();
            odrv->state = State::IDLE_UNHOMED;
            return true;
        }
        if (odrv->axis0_error & 0x1000) {
            Serial.println("Endstop detected! Switching to State::IDLE_UNHOMED and rehoming!");
            odrv->axis0_clear_errors();
            odrv->state = State::IDLE_UNHOMED;
            return true;
        }

        odrv->axis0_clear_errors();
        Serial.print("Error, switching back to state ");
        Serial.println((int32_t)state);
        return true;
    }
    return false;
}*/

void ignoreFor(uint32_t milliseconds) {
    ignoreStart = millis();
    ignorePeriod = milliseconds;
}

void updateProcess() {
    auto& rx = FLAME_Protocol::toMCU;
    auto& tx = FLAME_Protocol::toPC;
    auto odrv0 = &odrvData0;
    auto odrv1 = &odrvData1;

    if (millis() - ignoreStart < ignorePeriod) {
        return;
    }

    switch (state) {

    case State::ERROR:
        Serial.print("Error detected: ");
        Serial.println(odrv0->axis0_error);
        state = State::DISCONNECTED;
        odrv0->axis0_clear_errors++;
        odrv0->axis1_clear_errors++;
        ignoreFor(2000);
        return;

    case State::DISCONNECTED:
        odrv0->axis0_requested_state = AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION;
        odrv0->axis1_requested_state = AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION;
        state = State::WAIT_CALIBRATION_START;
        Serial.println("Starting calibration");
        break;
        
    case State::WAIT_CALIBRATION_START:
        if (odrv0->axis0_current_state == AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION && 
            odrv0->axis1_current_state == AXIS_STATE_ENCODER_INDEX_OFFSET_CALIBRATION) {
            state = State::CALIBRATION;
            Serial.println("Calibrating is going");
        }
        break;

    case State::CALIBRATION:
        if (odrv0->axis0_current_state == AXIS_STATE_IDLE && odrv0->axis1_current_state == AXIS_STATE_IDLE) {
            state = State::IDLE_UNHOMED;
            Serial.println("Finished calibrating motor");
        }
        break;

    case State::IDLE_UNHOMED:
        break;
    }

    processCount++;

    tx.odrive0Axis0Error = odrv0->axis0_error || odrv0->axis0_motor_error || odrv0->axis0_encoder_error || odrv0->axis0_controller_error;
    //tx.odrive0Axis1Error = odrv0->axis1_error || odrv0->axis1_motor_error || odrv0->axis1_encoder_error || odrv0->axis1_controller_error;
    tx.odrive0Error = tx.odrive0Axis0Error;// || tx.odrive0Axis1Error;

    tx.odrive1Axis0Error = odrv1->axis0_error || odrv1->axis0_motor_error || odrv1->axis0_encoder_error || odrv1->axis0_controller_error;
    //tx.odrive1Axis1Error = odrv1->axis1_error || odrv1->axis1_motor_error || odrv1->axis1_encoder_error || odrv1->axis1_controller_error;
    tx.odrive1Error = tx.odrive1Axis0Error;// || tx.odrive1Axis1Error;

    if (tx.odrive0Error || tx.odrive1Error) {
        state = State::ERROR;
    }

    if (odrv0->axis0_current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        if (tx.safetyMode) {
            odrv0->axis0_controller_config_input_mode = INPUT_MODE_TRAP_TRAJ;
        }
        else {
            odrv0->axis0_controller_config_input_mode = INPUT_MODE_POS_FILTER;
        }
    }
    else {
        tx.odrive0Error = true;
        tx.odrive1Error = true;
    }
    //Serial.println(odrv0->axis0_current_state);

    if (!tx.safetyMode) {
        odrv0->axis0_controller_input_pos = max(min(rx.desiredAxis1, AXIS0_MAX_ANGLE), AXIS0_MIN_ANGLE);
        //odrv0->axis1_controller_input_pos = max(min(rx.desiredAxis2, AXIS1_MAX_ANGLE), AXIS1_MIN_ANGLE);
        odrv1->axis0_controller_input_pos = max(min(rx.desiredAxis3, AXIS2_MAX_ANGLE), AXIS2_MIN_ANGLE);
        //odrv1->axis1_controller_input_pos = max(min(rx.desiredAxis4, AXIS3_MAX_ANGLE), AXIS3_MIN_ANGLE);
        tx.actualAxis1 = odrv0->axis0_encoder_pos_estimate;
        //tx.actualAxis2 = odrv0->axis1_encoder_pos_estimate;
        tx.actualAxis3 = odrv1->axis0_encoder_pos_estimate;
        //tx.actualAxis4 = odrv1->axis1_encoder_pos_estimate;
    }
    else {      // Safety mode
        rx.desiredAxis1 = odrv0->axis0_controller_input_pos;
        //rx.desiredAxis2 = odrv0->axis1_controller_input_pos;
        rx.desiredAxis3 = odrv1->axis0_controller_input_pos;
        //rx.desiredAxis4 = odrv1->axis1_controller_input_pos;
    }
}
