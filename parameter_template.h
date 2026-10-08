/*
 * parameter_template.h
 *
 * A reference list of every tuning constant in src/main.ino, grouped as in the sketch,
 * with the REFERENCE-ROBOT values. This file is NOT #included by main.ino. The sketch is
 * self-contained. Use this as a worksheet: copy your values into the matching
 * constants at the top of src/main.ino.
 *
 * Legend:
 *   [M] = USER MUST MEASURE on your robot
 *   [C] = calculated by the firmware from other constants
 *   [T] = tuned / empirical
 *   (p) = placeholder value in the sketch, not a measurement
 *
 * Pins are fixed. Do not change them (see hardware/PINOUT.md):
 *   IR_PIN = {14, 13, 16, 17, 18, 19, 23}   IR1 = GPIO14 DEFECTIVE, IR4 = GPIO17 = centre
 *   PWMA 25, AIN1 26, AIN2 27, STBY 5, BIN1 33, BIN2 4, PWMB 32, SDA 21, SCL 22
 *   PWM: 20000 Hz, 8 bit
 */
#pragma once
#include <stdint.h>

// ---- 1. robot geometry --------------------------------------------------------
static const bool     GEOMETRY_MEASURED = false;  // set true after measuring everything
static const uint16_t ROBOT_WIDTH_MM    = 190;    // [M] widest extent incl. sensor bar and wheels
static const uint16_t ROBOT_LENGTH_MM   = 250;    // [M] front-most to rear-most point
static const uint16_t TOF_SENSOR_TO_ROTATION_CENTER_MM = 90;  // [M] (p) sensor window to axle line
static const uint16_t SAFETY_MARGIN_MM  = 20;     // [T] clearance added everywhere
static const uint16_t OBSTACLE_WIDTH_MM = 100;    // [M] box width
static const uint16_t DRIVE_MM_PER_SEC  = 200;    // [M] (p) speed at AV_DRIVE_SPEED

// ---- 2. motors ----------------------------------------------------------------
static const bool  LEFT_INVERT   = false;
static const bool  RIGHT_INVERT  = false;
static const float LEFT_SCALE    = 1.00f;         // [T] trim
static const float RIGHT_SCALE   = 1.00f;

// ---- 3. line following --------------------------------------------------------
static const int   BASE_SPEED    = 140;           // [T] start lower when tuning (see docs/TUNING.md)
static const int   MIN_SPEED     = 75;
static const int   MAX_REVERSE   = 110;
static const float KP            = 32.0f;         // [T]
static const float KD            = 70.0f;         // [T] per control tick, not per second
static const float SLOWDOWN      = 14.0f;         // [T] speed lost per unit of |error|
static const uint32_t LOOP_US    = 4000;          // 4 ms = 250 Hz
static const uint32_t START_RAMP_MS = 400;

// ---- 4. lost line / gaps / end of track ---------------------------------------
static const uint32_t GAP_CROSS_MS     = 180;
static const int      GAP_SPEED        = 100;
static const int      SEARCH_SPEED     = 120;
static const uint32_t SEARCH_SWEEP_BASE_MS = 200;
static const uint32_t SEARCH_TIMEOUT_MS= 3500;

// ---- 5. finish bar (disabled) -------------------------------------------------
static const bool     ENABLE_FINISH_STOP = false;
static const uint32_t FINISH_HOLD_MS     = 120;
static const uint32_t FINISH_IGNORE_MS   = 1500;

// ---- 6. obstacle detection ----------------------------------------------------
static const uint16_t OBSTACLE_TRIGGER_MM    = 350;  // [C] effective = max(this, stop + 100) = 400
static const uint16_t OBSTACLE_STOP_MM       = 300;  // 0 = calculated; 300 is the reference
static const uint16_t OBSTACLE_EMERGENCY_MM  = 250;  // sketch default was 70
static const uint16_t OBSTACLE_CLEAR_MM      = 400;  // [C] effective = max(this, trigger + 40) = 440
static const bool     ALLOW_STOP_BELOW_CALCULATED_MINIMUM = false;
static const uint16_t STOP_BRAKE_ALLOWANCE_MM= 20;   // [T]
static const uint16_t MIN_APPROACH_ZONE_MM   = 100;
static const uint8_t  OBSTACLE_CONFIRM_COUNT = 3;
static const uint8_t  OBSTACLE_STOP_CONFIRM  = 2;
static const uint8_t  OBSTACLE_CLEAR_CONFIRM = 4;
static const uint8_t  OBSTACLE_EMERGENCY_CONFIRM = 2;
static const uint16_t TOF_MIN_VALID_MM       = 15;
static const uint16_t TOF_MAX_VALID_MM       = 1500;
static const uint32_t TOF_STALE_MS           = 400;
static const uint32_t TOF_STALE_HALT_MS      = 600;
static const int      APPROACH_SPEED         = 90;
static const uint32_t APPROACH_TIMEOUT_MS    = 5000;
static const uint32_t APPROACH_CANCEL_IGNORE_MS = 500;
static const uint32_t POST_DETOUR_IGNORE_MS  = 100;

// ---- 7. detour timings (time based) -------------------------------------------
static const int      AV_TURN_SPEED   = 130;
static const int      AV_DRIVE_SPEED  = 100;
static const int      AV_BACKUP_SPEED = 90;
static const uint32_t TURN_RIGHT_MS   = 420;   // [M] (p)
static const uint32_t TURN_LEFT_MS    = 420;   // [M] (p)
static const uint32_t RIGHT_SHIFT_MS  = 0;     // 0 = D_right / DRIVE_MM_PER_SEC * 1000 (825 ms)
static const uint32_t FORWARD_PASS_MS = 0;     // 0 = D_forward / DRIVE_MM_PER_SEC * 1000 (2725 ms)
static const uint32_t AV_SETTLE_MS    = 150;
static const uint32_t BACKUP_MAX_MS   = 1500;

// ---- 8. line reacquisition and alignment --------------------------------------
static const int      LINE_SEARCH_SPEED         = 80;
static const uint32_t LINE_SEARCH_TIMEOUT_MS    = 4000;
static const uint8_t  LINE_CONFIRM_MIN_SENSORS  = 1;
static const uint8_t  LINE_CONFIRM_READINGS     = 5;
static const uint32_t ALIGN_CREEP_MS            = 140;
static const uint32_t ALIGN_TURN_PERCENT        = 100;
static const int      ALIGN_FINE_SPEED          = 90;
static const float    CENTER_POSITION_TOLERANCE = 0.5f;
static const uint8_t  ALIGN_MAX_ACTIVE_SENSORS  = 3;
static const uint8_t  CENTER_CONFIRM_READINGS   = 5;
static const uint32_t ALIGN_TIMEOUT_MS          = 2500;
static const int      FAILSAFE_SEARCH_SPEED     = 90;
static const uint32_t FAILSAFE_SWEEP_BASE_MS    = 150;
static const uint32_t FAILSAFE_SEARCH_MS        = 5000;
static const int      RESUME_SPEED              = 100;
static const uint32_t RESUME_SLOW_MS            = 600;

// ---- 9. diagnostics -----------------------------------------------------------
static const uint16_t IR_TOGGLE_WARN_PER_SEC = 80;
