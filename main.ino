/*
 * ============================================================================
 *  BPUT Tech Carnival 2026 - T12 Robotics Competition
 *  7-sensor line follower + VL53L0X obstacle avoidance (always passes RIGHT)
 *  Board : ESP32 Dev Module   (Arduino IDE, ESP32 core 2.x or 3.x)
 *  Libs  : Adafruit VL53L0X   (Library Manager; installs Adafruit BusIO too)
 * ----------------------------------------------------------------------------
 *  BEFORE ANYTHING ELSE (hardware, not software):
 *    Turn the potentiometer on each of the six 3-pin IR modules until black and
 *    white give clean, reliable digital levels (the module LED should switch
 *    cleanly). Software polarity calibration only learns WHICH level means
 *    "line". It can NOT repair a badly adjusted sensor threshold.
 *    IR4 (GPIO17) is the 4-pin module: use its DO pin.
 *
 *  SENSOR BAR (17 mm pitch, positions from left edge of the 180 mm bar):
 *    IR1=39  IR2=56  IR3=73  IR4=90 (CENTRE)  IR5=107  IR6=124  IR7=141 mm
 *    20 mm line + 17 mm pitch => a correctly aligned robot sees 1 or 2 sensors.
 *
 *  HOW TO USE
 *   1. Put the robot on the track so that:
 *        - IR4 (centre sensor) is ON the line
 *        - IR1, IR2, IR6, IR7 are on the plain floor
 *   2. Press the BOOT button (on the ESP32 board) or send 's' in Serial Monitor
 *        -> robot calibrates line polarity (HIGH/LOW) automatically,
 *           blinks the LED for 1.5 s, then starts.
 *   3. Press BOOT (or send 's') again at any time to STOP.
 *
 *  OBSTACLE STATE MACHINE (non-blocking, runs inside runStep()):
 *    FOLLOW_LINE -> APPROACH_OBSTACLE -> STOP_FOR_OBSTACLE [-> BACKUP_TO_SAFE]
 *    -> TURN_RIGHT -> SHIFT_RIGHT -> TURN_FORWARD -> PASS_OBSTACLE
 *    -> TURN_TO_LINE -> SEARCH_LINE -> ALIGN_CREEP -> ALIGN_TURN -> ALIGN_FINE
 *    -> RESUME -> FOLLOW_LINE
 *    Failsafe: FAILSAFE_SEARCH (stopped, widening in-place sweep, then HALT).
 *
 *  SERIAL COMMANDS (115200 baud)
 *    s = start / stop      c = calibrate only     d = toggle debug stream
 *    t = motor test        v = print ToF distance for 3 s
 *    i = print raw/active IR sensors for 3 s
 *    g = print geometry / derived distances
 *    r / l = turn test (TURN_RIGHT_MS / TURN_LEFT_MS)
 *    f = shift test (RIGHT_SHIFT_MS)   p = pass test (FORWARD_PASS_MS)
 *    o = run the detour from the stop position (after calibration)
 *    h = help
 * ============================================================================
 *  REPOSITORY NOTE (not part of the original sketch)
 *    This file is the uploaded final sketch. Only four constants were changed
 *    to match the documented reference robot; each is tagged "CHANGED (repo)":
 *      ROBOT_WIDTH_MM 180->190, ROBOT_LENGTH_MM 160->250,
 *      OBSTACLE_STOP_MM 0->300, OBSTACLE_EMERGENCY_MM 70->250.
 *    No logic was modified. IR1 (GPIO14) is defective on the reference robot
 *    but this sketch still scans it - see docs/CODE_VS_BRIEF_MISMATCHES.md.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_VL53L0X.h>

// ============================== PIN MAP (DO NOT CHANGE) =====================
static const uint8_t IR_PIN[7] = {14, 13, 16, 17, 18, 19, 23};  // IR1..IR7 left->right
                                                               // IR1 = GPIO14, IR4 (CENTRE) = GPIO17

#define PIN_PWMA 25
#define PIN_AIN1 26
#define PIN_AIN2 27
#define PIN_STBY 5
#define PIN_BIN1 33
#define PIN_BIN2 4
#define PIN_PWMB 32

#define PIN_SDA  21
#define PIN_SCL  22

#define PIN_BTN  0     // BOOT button already on the ESP32 dev board (no wiring)
#define PIN_LED  2     // on-board LED (no wiring)

// ============================== PWM (core 2.x / 3.x safe) ===================
#define PWM_FREQ 20000
#define PWM_BITS 8
#define CH_A 0
#define CH_B 1

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, ch, duty) ledcWrite(pin, duty)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite(ch, duty)
#endif

// ============================================================================
//                    ALL COMPETITION TUNING PARAMETERS
// ============================================================================

// ---- 1. ROBOT GEOMETRY  (*** MEASURE THESE WITH A RULER ***) -----------------
// The values below are PLACEHOLDER ESTIMATES. Set GEOMETRY_MEASURED = true only
// after you have measured every one of them. While false, the robot prints a
// warning every time it starts.
static const bool     GEOMETRY_MEASURED = false;
static const uint16_t ROBOT_WIDTH_MM    = 190;   // widest extent incl. sensor bar (bar itself is 180 mm)  // CHANGED (repo): was 180 in the uploaded sketch
static const uint16_t ROBOT_LENGTH_MM   = 250;   // longest extent, front-most point to rear-most point    // CHANGED (repo): was 160 in the uploaded sketch
static const uint16_t TOF_SENSOR_TO_ROTATION_CENTER_MM = 90;  // *** MEASURE: VL53 sensor -> wheel axle (rotation centre) ***
static const uint16_t SAFETY_MARGIN_MM  = 20;    // M
static const uint16_t OBSTACLE_WIDTH_MM = 100;   // B (100 x 100 mm box)
static const uint16_t DRIVE_MM_PER_SEC  = 200;   // *** MEASURE: mm travelled in 1 s at AV_DRIVE_SPEED ***

// ---- 2. motors --------------------------------------------------------------
static const bool  LEFT_INVERT   = false;   // set true if LEFT  motor drives backwards
static const bool  RIGHT_INVERT  = false;   // set true if RIGHT motor drives backwards
static const float LEFT_SCALE    = 1.00f;   // trim: lower one side if robot drifts
static const float RIGHT_SCALE   = 1.00f;

// ---- 3. line following (UNCHANGED) ------------------------------------------
static const int   BASE_SPEED    = 140;     // 0..255 straight-line speed
static const int   MIN_SPEED     = 75;      // never slower than this while following
static const int   MAX_REVERSE   = 110;     // inner wheel may reverse this much in sharp turns
static const float KP            = 32.0f;
static const float KD            = 70.0f;
static const float SLOWDOWN      = 14.0f;   // speed lost per unit of |error|
static const uint32_t LOOP_US    = 4000;    // control period (4 ms = 250 Hz)
static const uint32_t START_RAMP_MS = 400;  // soft start

// ---- 4. lost line / gaps / end of track -------------------------------------
static const uint32_t GAP_CROSS_MS     = 180;   // drive straight this long over gaps
static const int      GAP_SPEED        = 100;
static const int      SEARCH_SPEED     = 120;   // pivot speed while searching
static const uint32_t SEARCH_SWEEP_BASE_MS = 200; // smallest sweep leg; each pass gets wider
static const uint32_t SEARCH_TIMEOUT_MS= 3500;  // give up -> STOP (end of track)

// ---- 5. FINISH (separate from line-loss logic) ------------------------------
//   ENABLE_FINISH_STOP is the ONLY switch for finish-bar detection. It is OFF.
//   End-of-track by line loss (SEARCH_TIMEOUT_MS above) is a separate mechanism.
static const bool     ENABLE_FINISH_STOP = false;
static const uint32_t FINISH_HOLD_MS     = 120;
static const uint32_t FINISH_IGNORE_MS   = 1500; // ignore start box after start

// ---- 6. obstacle DETECTION (VL53L0X, used only for obstacles) ---------------
//   OBSTACLE_STOP_MM = 0  -> use the CALCULATED safe stop distance (recommended).
//   OBSTACLE_STOP_MM > 0  -> your value, but never below the calculated minimum
//                            unless ALLOW_STOP_BELOW_CALCULATED_MINIMUM = true.
//   Calculated minimum safe turning distance (sensor -> box face) =
//        TOF_SENSOR_TO_ROTATION_CENTER_MM + corner radius + SAFETY_MARGIN_MM
//        corner radius = sqrt((ROBOT_LENGTH_MM/2)^2 + (ROBOT_WIDTH_MM/2)^2)
static const uint16_t OBSTACLE_TRIGGER_MM    = 350;  // begin slowing / approaching
static const uint16_t OBSTACLE_STOP_MM       = 300;  // 0 = calculated (see above)  // CHANGED (repo): was 0 in the uploaded sketch (0 would give 288 mm for L=250, W=190)
static const uint16_t OBSTACLE_EMERGENCY_MM  = 250;  // dangerously close -> brake / back off / halt  // CHANGED (repo): was 70 in the uploaded sketch
static const uint16_t OBSTACLE_CLEAR_MM      = 400;  // beyond this = obstacle gone (false alarm)
static const bool     ALLOW_STOP_BELOW_CALCULATED_MINIMUM = false;
static const uint16_t STOP_BRAKE_ALLOWANCE_MM= 20;   // overshoot allowance when braking from APPROACH_SPEED
static const uint16_t MIN_APPROACH_ZONE_MM   = 100;  // trigger is kept at least this far beyond the stop distance
static const uint8_t  OBSTACLE_CONFIRM_COUNT = 3;    // consecutive valid readings <= trigger
static const uint8_t  OBSTACLE_STOP_CONFIRM  = 2;    // consecutive valid readings <= stop distance
static const uint8_t  OBSTACLE_CLEAR_CONFIRM = 4;    // consecutive valid readings > clear distance
static const uint8_t  OBSTACLE_EMERGENCY_CONFIRM = 2;// consecutive valid readings <= emergency
static const uint16_t TOF_MIN_VALID_MM       = 15;   // readings outside [MIN, MAX] are ignored
static const uint16_t TOF_MAX_VALID_MM       = 1500;
static const uint32_t TOF_STALE_MS           = 400;  // no valid reading for this long = unknown
static const uint32_t TOF_STALE_HALT_MS      = 600;  // stale this long (braked) during approach -> HALT
static const int      APPROACH_SPEED         = 90;   // max wheel speed while approaching
static const uint32_t APPROACH_TIMEOUT_MS    = 5000; // approach never reached STOP -> cancel
static const uint32_t APPROACH_CANCEL_IGNORE_MS = 500;
static const uint32_t POST_DETOUR_IGNORE_MS  = 100;  // tiny; emergency stop is never ignored

// ---- 7. DETOUR TIMINGS (no wheel encoders -> time based, CALIBRATE ON YOUR FLOOR)
static const int      AV_TURN_SPEED   = 130;   // in-place turn speed (turn times depend on it!)
static const int      AV_DRIVE_SPEED  = 100;   // detour straight-drive speed
static const int      AV_BACKUP_SPEED = 90;    // reverse speed when too close to turn
static const uint32_t TURN_RIGHT_MS   = 420;   // *** time for a 90 deg RIGHT spin ***
static const uint32_t TURN_LEFT_MS    = 420;   // *** time for a 90 deg LEFT  spin ***
// 0 = compute from the geometry and DRIVE_MM_PER_SEC. After measuring, type a number.
static const uint32_t RIGHT_SHIFT_MS  = 0;     // *** sideways shift time ***
static const uint32_t FORWARD_PASS_MS = 0;     // *** forward pass time ***
static const uint32_t AV_SETTLE_MS    = 150;   // brake pause between detour steps
static const uint32_t BACKUP_MAX_MS   = 1500;  // max reversing time

// ---- 8. LINE REACQUISITION & ALIGNMENT --------------------------------------
//   "Line found" and "robot aligned" use DIFFERENT criteria.
static const int      LINE_SEARCH_SPEED         = 80;    // slow creep toward the line
static const uint32_t LINE_SEARCH_TIMEOUT_MS    = 4000;  // max time creeping toward the line
static const uint8_t  LINE_CONFIRM_MIN_SENSORS  = 1;     // FOUND: 1 sensor is enough ...
static const uint8_t  LINE_CONFIRM_READINGS     = 5;     // ... for this many consecutive 4 ms ticks
static const uint32_t ALIGN_CREEP_MS            = 140;   // roll on so the wheel axle is over the line
static const uint32_t ALIGN_TURN_PERCENT        = 100;   // % of TURN_RIGHT_MS for the alignment turn
static const int      ALIGN_FINE_SPEED          = 90;    // slow corrective spin
static const float    CENTER_POSITION_TOLERANCE = 0.5f;  // ALIGNED: |pos| <= this (units = sensor spacings)
static const uint8_t  ALIGN_MAX_ACTIVE_SENSORS  = 3;     // ALIGNED: more active sensors = bar lies along the line
static const uint8_t  CENTER_CONFIRM_READINGS   = 5;     // ALIGNED: stable for this many consecutive ticks
static const uint32_t ALIGN_TIMEOUT_MS          = 2500;  // fine alignment limit -> failsafe
static const int      FAILSAFE_SEARCH_SPEED     = 90;    // in-place sweep speed
static const uint32_t FAILSAFE_SWEEP_BASE_MS    = 150;   // smallest sweep leg; each pass gets wider
static const uint32_t FAILSAFE_SEARCH_MS        = 5000;  // sweep time, then HALT for good
static const int      RESUME_SPEED              = 100;   // speed cap right after rejoining ...
static const uint32_t RESUME_SLOW_MS            = 600;   // ... for this long

// ---- 9. diagnostics ----------------------------------------------------------
static const uint16_t IR_TOGGLE_WARN_PER_SEC = 80;       // debug mode: sensor flipping faster = suspect

// ============================== STATE =======================================
#define ST_IDLE 0
#define ST_RUN  1

// obstacle state machine states
#define AV_FOLLOW          0
#define AV_APPROACH        1
#define AV_STOP            2
#define AV_BACKUP          3
#define AV_TURN_RIGHT      4
#define AV_SHIFT_RIGHT     5
#define AV_TURN_FORWARD    6
#define AV_PASS_OBSTACLE   7
#define AV_TURN_TO_LINE    8
#define AV_SEARCH_LINE     9
#define AV_ALIGN_CREEP     10
#define AV_ALIGN_TURN      11
#define AV_ALIGN_FINE      12
#define AV_FAILSAFE_SEARCH 13
#define AV_RESUME          14

static const char *const AV_NAME[] = {
  "FOLLOW_LINE", "APPROACH_OBSTACLE", "STOP_FOR_OBSTACLE", "BACKUP_TO_SAFE", "TURN_RIGHT",
  "SHIFT_RIGHT", "TURN_FORWARD", "PASS_OBSTACLE", "TURN_TO_LINE", "SEARCH_LINE",
  "ALIGN_CREEP", "ALIGN_TURN", "ALIGN_FINE", "FAILSAFE_SEARCH", "RESUME"
};

static const uint8_t IR4_BIT = (1 << 3);       // IR4 = CENTRE sensor

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- ToF data shared between the ToF task (core 0) and loop() (core 1) ------
// The task writes t_* ONLY inside a critical section; loop() copies them into
// the g_* variables with tofRefresh() once per control tick, so every decision
// in one tick uses one consistent snapshot.
static portMUX_TYPE g_tofMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t t_dist = 8190;
static uint32_t t_time = 0;       // time of last VALID reading
static uint8_t  t_close = 0;      // consecutive valid readings <= trigger
static uint8_t  t_stop  = 0;      // consecutive valid readings <= stop distance
static uint8_t  t_clear = 0;      // consecutive valid readings >  clear distance
static uint8_t  t_emg   = 0;      // consecutive valid readings <= emergency
uint16_t g_dist = 8190;
uint32_t g_distTime = 0;
uint8_t  g_closeCnt = 0, g_stopCnt = 0, g_clearCnt = 0, g_emgCnt = 0;
bool g_tofOk = false;

// derived geometry (computed once in computeGeometry(), read-only afterwards)
uint16_t g_cornerRadiusMm = 0;
uint16_t g_minSafeStopMm  = 0;   // calculated: sensor->box distance needed to rotate safely
uint16_t g_turnMinMm      = 0;   // below this at STOP => back up before turning
uint16_t g_stopEffMm      = 0;   // distance at which the robot stops (SAFE_STOP_DISTANCE_FROM_TOF)
uint16_t g_triggerEffMm   = 0;
uint16_t g_clearEffMm     = 0;
uint16_t g_emergencyEffMm = 0;
uint16_t g_dRightMm       = 0;
uint16_t g_dForwardMm     = 0;
uint32_t g_rightShiftMs   = 0;
uint32_t g_forwardPassMs  = 0;

bool    g_lineHigh[7] = {true, true, true, true, true, true, true}; // sensor reads HIGH on line?
bool    g_calibrated  = false;
uint8_t g_mask = 0;      // bit i = sensor i sees the line
uint8_t g_cnt  = 0;
float   g_pos  = 0.0f;   // -3 (far left) .. +3 (far right)
uint16_t g_toggle[7] = {0};
uint32_t g_lastToggleCheck = 0;

uint8_t  g_state       = ST_IDLE;
bool     g_debug       = false;
uint32_t g_runStart    = 0;
uint32_t g_rampStart   = 0;
uint32_t g_lastTick    = 0;
uint32_t g_lastSeen    = 0;
float    g_lastErr     = 0.0f;
float    g_dFilt       = 0.0f;
float    g_lastPos     = 0.0f;
int      g_lastDir     = +1;
bool     g_lostFlag    = false;
uint32_t g_ignoreObstUntil = 0;
uint32_t g_resumeCapUntil  = 0;
uint32_t g_finishStart = 0;
uint16_t g_avoidCount  = 0;
uint32_t g_lastDebug   = 0;

// obstacle state machine
uint8_t  g_avState     = AV_FOLLOW;
uint32_t g_avT0        = 0;       // start time of current state (future value = settling)
uint8_t  g_stable      = 0;       // consecutive-reading counter used by SEARCH/ALIGN
int      g_alignDir    = +1;      // spin direction used by ALIGN_FINE
uint32_t g_staleStart  = 0;       // when ToF went stale during approach (0 = not stale)
float    g_speedCap    = 255.0f;  // follower speed cap (lowered while approaching)
bool     g_testDetour  = false;   // 'o' command: stop after one detour

// ============================== SMALL HELPERS ===============================
static int imax(int a, int b) { return a > b ? a : b; }

static void computeGeometry() {
  float hl = ROBOT_LENGTH_MM / 2.0f;
  float hw = ROBOT_WIDTH_MM / 2.0f;
  g_cornerRadiusMm = (uint16_t)ceilf(sqrtf(hl * hl + hw * hw));
  g_minSafeStopMm  = (uint16_t)(TOF_SENSOR_TO_ROTATION_CENTER_MM + g_cornerRadiusMm + SAFETY_MARGIN_MM);

  if (ALLOW_STOP_BELOW_CALCULATED_MINIMUM && OBSTACLE_STOP_MM > 0) {
    g_stopEffMm = OBSTACLE_STOP_MM;
    g_turnMinMm = (OBSTACLE_STOP_MM > STOP_BRAKE_ALLOWANCE_MM) ? (OBSTACLE_STOP_MM - STOP_BRAKE_ALLOWANCE_MM) : 0;
  } else {
    int want = g_minSafeStopMm + STOP_BRAKE_ALLOWANCE_MM;
    g_stopEffMm = (uint16_t)imax(OBSTACLE_STOP_MM, want);
    g_turnMinMm = g_minSafeStopMm;
  }
  g_triggerEffMm = (uint16_t)imax(OBSTACLE_TRIGGER_MM, g_stopEffMm + MIN_APPROACH_ZONE_MM);
  g_clearEffMm   = (uint16_t)imax(OBSTACLE_CLEAR_MM, g_triggerEffMm + 40);
  g_emergencyEffMm = OBSTACLE_EMERGENCY_MM;
  if (g_emergencyEffMm >= g_stopEffMm) g_emergencyEffMm = (g_stopEffMm > 20) ? (g_stopEffMm - 20) : 1;

  g_dRightMm   = (uint16_t)(OBSTACLE_WIDTH_MM / 2 + ROBOT_WIDTH_MM / 2 + SAFETY_MARGIN_MM);
  g_dForwardMm = (uint16_t)(g_stopEffMm + OBSTACLE_WIDTH_MM + ROBOT_LENGTH_MM / 2 + SAFETY_MARGIN_MM);

  uint32_t speed = (uint32_t)imax(DRIVE_MM_PER_SEC, 1);
  g_rightShiftMs  = RIGHT_SHIFT_MS  ? RIGHT_SHIFT_MS  : ((uint32_t)g_dRightMm   * 1000UL) / speed;
  g_forwardPassMs = FORWARD_PASS_MS ? FORWARD_PASS_MS : ((uint32_t)g_dForwardMm * 1000UL) / speed;
}

static void printGeometry() {
  Serial.println(F("--- GEOMETRY (estimates until you measure/calibrate) ---"));
  Serial.printf("corner radius        = %u mm\n", (unsigned)g_cornerRadiusMm);
  Serial.printf("min safe turn dist   = %u mm  (ToF offset %u + corner %u + margin %u)\n",
                (unsigned)g_minSafeStopMm, (unsigned)TOF_SENSOR_TO_ROTATION_CENTER_MM,
                (unsigned)g_cornerRadiusMm, (unsigned)SAFETY_MARGIN_MM);
  Serial.printf("stop distance        = %u mm | trigger = %u mm | clear = %u mm | emergency = %u mm\n",
                (unsigned)g_stopEffMm, (unsigned)g_triggerEffMm, (unsigned)g_clearEffMm, (unsigned)g_emergencyEffMm);
  Serial.printf("D_right   = %u mm -> RIGHT_SHIFT_MS  = %lu%s\n", (unsigned)g_dRightMm,
                (unsigned long)g_rightShiftMs, RIGHT_SHIFT_MS ? " (manual)" : " (calculated)");
  Serial.printf("D_forward = %u mm -> FORWARD_PASS_MS = %lu%s\n", (unsigned)g_dForwardMm,
                (unsigned long)g_forwardPassMs, FORWARD_PASS_MS ? " (manual)" : " (calculated)");
  if (!GEOMETRY_MEASURED)
    Serial.println(F("WARNING: robot geometry values are PLACEHOLDERS. Measure them, then set GEOMETRY_MEASURED = true."));
}

// ============================== MOTORS ======================================
static void driveMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t ch,
                       int spd, bool invert, float scale) {
  spd = (int)(spd * scale);
  spd = constrain(spd, -255, 255);
  if (invert) spd = -spd;
  if (spd > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (spd < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  PWM_WRITE(pwmPin, ch, abs(spd));
}

static void setMotors(int left, int right) {
  digitalWrite(PIN_STBY, HIGH);
  driveMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_A, left,  LEFT_INVERT,  LEFT_SCALE);
  driveMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_B, right, RIGHT_INVERT, RIGHT_SCALE);
}

static void motorsStop(bool brake) {
  if (brake) {   // TB6612 short brake: IN1 = IN2 = HIGH
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
    PWM_WRITE(PIN_PWMA, CH_A, 255);
    PWM_WRITE(PIN_PWMB, CH_B, 255);
  } else {
    setMotors(0, 0);
  }
}

// ============================== INPUTS ======================================
static bool buttonEvent() {
  static bool last = HIGH;
  static uint32_t lastChange = 0;
  bool cur = digitalRead(PIN_BTN);
  uint32_t now = millis();
  if (cur != last && (now - lastChange) > 40) {
    lastChange = now;
    last = cur;
    if (cur == LOW) return true;
  }
  return false;
}

// true if the user asked to stop (button or serial 's'/'x'); also sets IDLE
static bool userStopRequested() {
  bool stop = buttonEvent();
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S' || c == 'x' || c == 'X') stop = true;
  }
  if (stop) {
    g_state = ST_IDLE; g_avState = AV_FOLLOW; g_speedCap = 255.0f; g_resumeCapUntil = 0;
    motorsStop(true);
  }
  return stop;
}

// ============================== LINE SENSORS ================================
static void readSensors() {
  static const float W[7] = {-3, -2, -1, 0, 1, 2, 3};
  static uint8_t prevMask = 0;
  uint8_t m = 0;
  uint8_t c = 0;
  float sum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    bool isHigh = (digitalRead(IR_PIN[i]) == HIGH);
    if (isHigh == g_lineHigh[i]) {
      m |= (1 << i);
      c++;
      sum += W[i];
    }
  }
  uint8_t diff = m ^ prevMask;                       // toggle counter (diagnostics)
  if (diff) {
    for (uint8_t i = 0; i < 7; i++)
      if ((diff & (1 << i)) && g_toggle[i] < 65000) g_toggle[i]++;
  }
  prevMask = m;
  g_mask = m;
  g_cnt  = c;
  if (c) g_pos = sum / c;
}

// ALIGNED = IR4 sees the line, the line position is near centre, and the bar
// is not lying along the line (too many sensors active). Reads the sensors.
static bool centerAligned() {
  readSensors();
  return (g_mask & IR4_BIT) && (g_cnt <= ALIGN_MAX_ACTIVE_SENSORS) &&
         (fabsf(g_pos) <= CENTER_POSITION_TOLERANCE);
}

// Automatic polarity calibration (robot stationary on the line, IR4 on line).
// IR4 (4-pin module) is measured directly; IR1-IR3/IR5-IR6 share the 3-pin
// module polarity; IR7 is measured separately, so any mix of modules works.
// NOTE: the potentiometers must already be adjusted so black/white switch the
// digital output reliably. Software cannot fix a badly set threshold.
static bool calibrate() {
  motorsStop(false);
  Serial.println(F("\n--- CALIBRATION (IR4 on line, IR1/2/6/7 on floor) ---"));
  Serial.println(F("(Pot on each IR module must already be set for clean black/white switching)"));

  uint16_t hi[7] = {0};
  uint16_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    for (uint8_t i = 0; i < 7; i++) hi[i] += (digitalRead(IR_PIN[i]) == HIGH);
    n++;
    delay(1);
  }
  bool st[7];
  for (uint8_t i = 0; i < 7; i++) {
    st[i] = (hi[i] * 2 > n);
    Serial.printf("IR%d (GPIO%d) reads %s  (%u/%u HIGH)\n", i + 1, IR_PIN[i], st[i] ? "HIGH" : "LOW", hi[i], n);
    if (hi[i] > n / 10 && hi[i] < n - n / 10)
      Serial.printf("  WARNING: IR%d (GPIO%d) is unstable - adjust its pot / check wiring\n", i + 1, IR_PIN[i]);
  }

  // Background level of the IR1-IR6 module family = majority of IR1, IR2, IR6
  uint8_t votes = st[0] + st[1] + st[5];
  bool bgA = (votes >= 2);
  if (st[0] != bgA || st[1] != bgA || st[5] != bgA)
    Serial.println(F("  WARNING: IR1/IR2/IR6 disagree - one may be near the line or badly adjusted"));

  if (st[3] == bgA) {
    Serial.println(F("CALIBRATION FAILED: IR4 reads the same as the floor."));
    Serial.println(F("Place the robot with the line directly under IR4 and retry."));
    return false;
  }

  bool lineA = !bgA;                 // level of IR1-IR6 on the line
  for (uint8_t i = 0; i < 6; i++) g_lineHigh[i] = lineA;
  g_lineHigh[3] = st[3];             // IR4 is measured directly on the line
  g_lineHigh[6] = !st[6];            // IR7 is separately handled (assumed on floor)
  if (st[6] != bgA) Serial.println(F("  NOTE: IR7 has opposite polarity - handled automatically"));

  g_calibrated = true;
  Serial.print(F("Line level per sensor (1=HIGH on line): "));
  for (uint8_t i = 0; i < 7; i++) Serial.print(g_lineHigh[i] ? '1' : '0');
  Serial.println(F("\nCALIBRATION OK"));
  return true;
}

// ============================== TOF (core 0 task) ===========================
// Invalid / out-of-range readings are IGNORED (they change no counter and do
// not refresh t_time). Only valid readings feed the consecutive counters.
static void tofTask(void *pv) {
  VL53L0X_RangingMeasurementData_t m;
  for (;;) {
    lox.rangingTest(&m, false);
    if (m.RangeStatus != 4 && m.RangeMilliMeter >= TOF_MIN_VALID_MM && m.RangeMilliMeter <= TOF_MAX_VALID_MM) {
      uint16_t d = m.RangeMilliMeter;
      uint32_t nowMs = millis();
      portENTER_CRITICAL(&g_tofMux);
      t_dist = d;
      t_time = nowMs;
      if (d <= g_triggerEffMm)   { if (t_close < 250) t_close++; } else t_close = 0;
      if (d <= g_stopEffMm)      { if (t_stop  < 250) t_stop++;  } else t_stop  = 0;
      if (d >  g_clearEffMm)     { if (t_clear < 250) t_clear++; } else t_clear = 0;
      if (d <= g_emergencyEffMm) { if (t_emg   < 250) t_emg++;   } else t_emg   = 0;
      portEXIT_CRITICAL(&g_tofMux);
    }
    vTaskDelay(1);
  }
}

// copy the task's data into this task's own variables (one consistent snapshot)
static void tofRefresh() {
  portENTER_CRITICAL(&g_tofMux);
  g_dist = t_dist; g_distTime = t_time;
  g_closeCnt = t_close; g_stopCnt = t_stop; g_clearCnt = t_clear; g_emgCnt = t_emg;
  portEXIT_CRITICAL(&g_tofMux);
}

// forget old counts so stale readings can never trigger a decision
static void tofResetCounters() {
  portENTER_CRITICAL(&g_tofMux);
  t_close = 0; t_stop = 0; t_clear = 0; t_emg = 0;
  portEXIT_CRITICAL(&g_tofMux);
  g_closeCnt = 0; g_stopCnt = 0; g_clearCnt = 0; g_emgCnt = 0;
}

static bool tofFresh() { return g_tofOk && ((uint32_t)(millis() - g_distTime) < TOF_STALE_MS); }
static bool obstacleAhead() { return tofFresh() && g_closeCnt >= OBSTACLE_CONFIRM_COUNT; }
static bool emergencyNow()  { return tofFresh() && g_emgCnt >= OBSTACLE_EMERGENCY_CONFIRM; }

// ============================== HELPERS =====================================
static void haltRobot(const char *msg) {
  motorsStop(true);
  g_state = ST_IDLE;
  g_avState = AV_FOLLOW;
  g_speedCap = 255.0f;
  g_resumeCapUntil = 0;
  g_testDetour = false;
  digitalWrite(PIN_LED, LOW);
  Serial.print(F("STOP: "));
  Serial.println(msg);
}

static void resetControl() {
  g_lastErr = 0; g_dFilt = 0; g_lostFlag = false;
  g_lastSeen = millis();
  g_ignoreObstUntil = millis();
  g_finishStart = 0;
  g_lastTick = micros();
}

// blocking timed drive, used ONLY by the serial calibration/test commands
static bool timedRun(int l, int r, uint32_t ms) {
  uint32_t t0 = millis();
  setMotors(l, r);
  while (millis() - t0 < ms) {
    if (userStopRequested()) return false;
    delay(2);
  }
  motorsStop(true);
  return true;
}

// Progressively widening sweep. Returns the spin direction (+1 right, -1 left)
// at time t (ms since the sweep began). With A_k = k*base and s = firstDir:
//   s*A1 , -s*2*A1 , s*(A1+A2) , -s*2*A2 , s*(A2+A3) , -s*2*A3 , ...
// i.e. small first-side, small other-side, larger first-side, larger other-side...
static int sweepDir(uint32_t t, int firstDir, uint32_t base) {
  uint32_t acc = 0;
  for (uint32_t k = 1; k <= 20; k++) {
    uint32_t A = k * base;
    acc += (k == 1) ? A : ((k - 1) * base + A);    // out-leg
    if (t < acc) return firstDir;
    acc += 2 * A;                                   // back-leg
    if (t < acc) return -firstDir;
  }
  return firstDir;
}

// ============================== LINE FOLLOWING ==============================
// Same algorithm and constants. Differences: (1) base speed is limited by
// g_speedCap / the post-detour cap, (2) the lost-line search now really widens.
static void followLine(uint32_t now) {
  readSensors();

  // optional finish bar (separate from line-loss logic)
  if (ENABLE_FINISH_STOP && (now - g_runStart) > FINISH_IGNORE_MS) {
    if (g_cnt == 7) {
      if (g_finishStart == 0) g_finishStart = now;
      else if (now - g_finishStart >= FINISH_HOLD_MS) { haltRobot("finish bar detected"); return; }
    } else g_finishStart = 0;
  }

  float ramp = 1.0f;
  if (now - g_rampStart < START_RAMP_MS)
    ramp = 0.5f + 0.5f * (float)(now - g_rampStart) / (float)START_RAMP_MS;

  if (g_cnt > 0) {
    float err = g_pos;
    g_lastSeen = now;
    g_lastPos = err;
    if (err < -0.5f) g_lastDir = -1;
    else if (err > 0.5f) g_lastDir = +1;

    if (g_lostFlag) { g_lastErr = err; g_dFilt = 0; g_lostFlag = false; }
    float dRaw = err - g_lastErr;
    g_dFilt = 0.5f * g_dFilt + 0.5f * dRaw;
    g_lastErr = err;

    float corr = KP * err + KD * g_dFilt;
    float base = BASE_SPEED - SLOWDOWN * fabsf(err);
    if (base < MIN_SPEED) base = MIN_SPEED;
    base *= ramp;
    float cap = g_speedCap;                                   // obstacle approach limit
    if ((int32_t)(g_resumeCapUntil - now) > 0 && cap > RESUME_SPEED) cap = RESUME_SPEED;  // post-detour limit
    if (base > cap) base = cap;

    int l = (int)(base + corr);
    int r = (int)(base - corr);
    l = constrain(l, -MAX_REVERSE, 255);
    r = constrain(r, -MAX_REVERSE, 255);
    setMotors(l, r);
  } else {
    g_lostFlag = true;
    uint32_t lost = now - g_lastSeen;
    if (lost < GAP_CROSS_MS) {
      if (fabsf(g_lastPos) >= 2.0f)             // line left via the edge: sharp turn
        setMotors(g_lastDir * SEARCH_SPEED, -g_lastDir * SEARCH_SPEED);
      else                                      // probably a gap: go straight
        setMotors(GAP_SPEED, GAP_SPEED);
    } else {
      uint32_t t = lost - GAP_CROSS_MS;
      if (t > SEARCH_TIMEOUT_MS) { haltRobot("line lost - end of track"); return; }
      int dir = sweepDir(t, g_lastDir, SEARCH_SWEEP_BASE_MS);   // widening sweep
      setMotors(dir * SEARCH_SPEED, -dir * SEARCH_SPEED);
    }
  }
}

// ============================== OBSTACLE STATE MACHINE ======================
// Enter a state. Every state except FOLLOW_LINE and APPROACH_OBSTACLE is
// preceded by a short brake pause (AV_SETTLE_MS): g_avT0 is set in the future
// and avStep() just waits (motors braked) until it arrives.
static void avGoto(uint8_t s) {
  uint32_t now = millis();
  g_avState = s;
  g_avT0 = now;
  g_stable = 0;
  g_staleStart = 0;
  g_speedCap = (s == AV_APPROACH) ? (float)APPROACH_SPEED : 255.0f;
  if (s == AV_ALIGN_FINE) g_alignDir = +1;
  if (s != AV_APPROACH) tofResetCounters();        // old readings may never trigger a new decision
  if (s != AV_FOLLOW && s != AV_APPROACH) {
    motorsStop(true);
    g_avT0 = now + AV_SETTLE_MS;
  }
  Serial.print(F("AV -> "));
  Serial.println(AV_NAME[s]);
}

static void avStep(uint32_t now) {
  int32_t el = (int32_t)(now - g_avT0);
  if (el < 0) return;                       // settling: motors are braked
  uint32_t e = (uint32_t)el;

  switch (g_avState) {

    case AV_APPROACH:                       // keep following the line, slowly
      if (emergencyNow()) {                 // dangerously close: brake, then STOP decides
        Serial.println(F("EMERGENCY distance during approach"));
        avGoto(AV_STOP);
        break;
      }
      if (!tofFresh()) {                    // distance unknown: never drive blind
        motorsStop(true);
        if (g_staleStart == 0) g_staleStart = now;
        if (now - g_staleStart > TOF_STALE_HALT_MS) haltRobot("ToF stale during approach");
        break;
      }
      g_staleStart = 0;
      followLine(now);
      if (g_state != ST_RUN) break;         // follower may have halted the robot
      if (g_stopCnt >= OBSTACLE_STOP_CONFIRM) {
        avGoto(AV_STOP);
      } else if (g_clearCnt >= OBSTACLE_CLEAR_CONFIRM) {          // false alarm
        avGoto(AV_FOLLOW);
      } else if (e > APPROACH_TIMEOUT_MS) {
        avGoto(AV_FOLLOW);
        g_ignoreObstUntil = now + APPROACH_CANCEL_IGNORE_MS;
      }
      break;

    case AV_STOP:                           // stopped (braked during settle)
      if (!tofFresh()) { haltRobot("ToF stale at obstacle stop"); break; }
      if (g_dist < g_turnMinMm) {           // too close to rotate without hitting it
        Serial.printf("Too close to turn (%u mm < %u mm) - backing up\n", (unsigned)g_dist, (unsigned)g_turnMinMm);
        avGoto(AV_BACKUP);
        break;
      }
      g_avoidCount++;
      Serial.printf("Obstacle #%u, stopped at %u mm\n", g_avoidCount, (unsigned)g_dist);
      avGoto(AV_TURN_RIGHT);
      break;

    case AV_BACKUP:                         // reverse until the stop distance is restored
      if (!tofFresh()) { haltRobot("ToF stale while backing up"); break; }
      if (g_dist >= g_stopEffMm) { avGoto(AV_STOP); break; }
      if (e >= BACKUP_MAX_MS) { haltRobot("could not back up to a safe turning distance"); break; }
      setMotors(-AV_BACKUP_SPEED, -AV_BACKUP_SPEED);
      break;

    case AV_TURN_RIGHT:                     // ~90 deg RIGHT
      setMotors(AV_TURN_SPEED, -AV_TURN_SPEED);
      if (e >= TURN_RIGHT_MS) avGoto(AV_SHIFT_RIGHT);
      break;

    case AV_SHIFT_RIGHT:                    // sideways clearance (to the right)
      if (emergencyNow()) { haltRobot("object too close during SHIFT_RIGHT"); break; }
      setMotors(AV_DRIVE_SPEED, AV_DRIVE_SPEED);
      if (e >= g_rightShiftMs) avGoto(AV_TURN_FORWARD);
      break;

    case AV_TURN_FORWARD:                   // ~90 deg LEFT = original heading
      setMotors(-AV_TURN_SPEED, AV_TURN_SPEED);
      if (e >= TURN_LEFT_MS) avGoto(AV_PASS_OBSTACLE);
      break;

    case AV_PASS_OBSTACLE:                  // drive past the box
      if (emergencyNow()) { haltRobot("object too close during PASS_OBSTACLE"); break; }
      setMotors(AV_DRIVE_SPEED, AV_DRIVE_SPEED);
      if (e >= g_forwardPassMs) avGoto(AV_TURN_TO_LINE);
      break;

    case AV_TURN_TO_LINE:                   // ~90 deg LEFT, facing the line
      setMotors(-AV_TURN_SPEED, AV_TURN_SPEED);
      if (e >= TURN_LEFT_MS) avGoto(AV_SEARCH_LINE);
      break;

    case AV_SEARCH_LINE:                    // creep, read all 7 IR sensors
      if (emergencyNow()) { haltRobot("object too close during SEARCH_LINE"); break; }
      setMotors(LINE_SEARCH_SPEED, LINE_SEARCH_SPEED);
      readSensors();
      if (g_cnt >= LINE_CONFIRM_MIN_SENSORS) { if (g_stable < 250) g_stable++; } else g_stable = 0;
      if (g_stable >= LINE_CONFIRM_READINGS) avGoto(AV_ALIGN_CREEP);        // line FOUND
      else if (e >= LINE_SEARCH_TIMEOUT_MS) avGoto(AV_FAILSAFE_SEARCH);
      break;

    case AV_ALIGN_CREEP:                    // roll on so the wheel axle is over the line
      setMotors(LINE_SEARCH_SPEED, LINE_SEARCH_SPEED);
      if (e >= ALIGN_CREEP_MS) avGoto(AV_ALIGN_TURN);
      break;

    case AV_ALIGN_TURN:                     // ~90 deg RIGHT alignment turn
      setMotors(AV_TURN_SPEED, -AV_TURN_SPEED);
      if (e >= (TURN_RIGHT_MS * ALIGN_TURN_PERCENT) / 100) avGoto(AV_ALIGN_FINE);
      break;

    case AV_ALIGN_FINE: {                   // IR4 on line + centred + stable
      if (centerAligned()) {
        motorsStop(true);
        if (g_stable < 250) g_stable++;
        if (g_stable >= CENTER_CONFIRM_READINGS) { avGoto(AV_RESUME); break; }
      } else {
        g_stable = 0;
        if (g_cnt > 0 && g_cnt <= ALIGN_MAX_ACTIVE_SENSORS) {
          if (g_pos > 0.25f) g_alignDir = +1;          // line is to the right -> turn right
          else if (g_pos < -0.25f) g_alignDir = -1;    // line is to the left  -> turn left
        } else {
          g_alignDir = +1;                             // nothing / crossing sideways: keep turning right
        }
        setMotors(g_alignDir * ALIGN_FINE_SPEED, -g_alignDir * ALIGN_FINE_SPEED);
      }
      if (g_avState == AV_ALIGN_FINE && e >= ALIGN_TIMEOUT_MS) avGoto(AV_FAILSAFE_SEARCH);
      break;
    }

    case AV_FAILSAFE_SEARCH: {              // stopped first, then widening in-place sweep
      if (centerAligned()) {
        motorsStop(true);
        if (g_stable < 250) g_stable++;
        if (g_stable >= CENTER_CONFIRM_READINGS) { avGoto(AV_RESUME); break; }
      } else {
        g_stable = 0;
        int dir = sweepDir(e, +1, FAILSAFE_SWEEP_BASE_MS);
        setMotors(dir * FAILSAFE_SEARCH_SPEED, -dir * FAILSAFE_SEARCH_SPEED);
      }
      if (g_avState == AV_FAILSAFE_SEARCH && e >= FAILSAFE_SEARCH_MS) haltRobot("line not reacquired - sweep timed out");
      break;
    }

    case AV_RESUME:                         // hand control back to the PD follower
      if (g_testDetour) { haltRobot("detour test finished"); break; }
      resetControl();                       // clears PD history, lost-line flag, lastSeen
      g_rampStart = now;                    // soft-start ramp again
      g_resumeCapUntil = now + RESUME_SLOW_MS;
      avGoto(AV_FOLLOW);
      g_ignoreObstUntil = now + POST_DETOUR_IGNORE_MS;
      Serial.println(F("Line re-acquired and aligned - following resumed."));
      break;

    default:
      avGoto(AV_FOLLOW);
      break;
  }
}

static void runStep() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - g_lastTick) < LOOP_US) return;
  g_lastTick = nowUs;
  uint32_t now = millis();
  tofRefresh();

  if (g_avState == AV_FOLLOW) {
    bool trigger = emergencyNow() || (((int32_t)(now - g_ignoreObstUntil) > 0) && obstacleAhead());
    if (trigger) {
      Serial.printf("Obstacle detected at %u mm - approaching slowly\n", (unsigned)g_dist);
      avGoto(AV_APPROACH);
    } else {
      followLine(now);                      // normal PD line following
    }
  } else {
    avStep(now);                            // obstacle detour state machine
  }

  if (g_debug && now - g_lastDebug > 100) {
    g_lastDebug = now;
    Serial.printf("[%s] mask=", AV_NAME[g_avState]);
    for (int i = 0; i < 7; i++) Serial.print((g_mask >> i) & 1);
    Serial.printf(" cnt=%u pos=%.2f dist=%u fresh=%d close=%u stop=%u emg=%u\n",
                  g_cnt, g_pos, (unsigned)g_dist, (int)tofFresh(), g_closeCnt, g_stopCnt, g_emgCnt);
  }
  if (g_debug && now - g_lastToggleCheck >= 1000) {          // suspect-sensor warning (debug only)
    g_lastToggleCheck = now;
    for (uint8_t i = 0; i < 7; i++) {
      if (g_toggle[i] > IR_TOGGLE_WARN_PER_SEC)
        Serial.printf("WARNING: IR%d (GPIO%d) toggled %u times in 1 s - noisy/faulty input? (GPIO map unchanged)\n",
                      i + 1, IR_PIN[i], (unsigned)g_toggle[i]);
      g_toggle[i] = 0;
    }
  }
}

// ============================== COMMANDS ====================================
static void errorBlink() {
  for (int i = 0; i < 10; i++) { digitalWrite(PIN_LED, i & 1); delay(80); }
  digitalWrite(PIN_LED, LOW);
}

static void startSequence() {
  if (!GEOMETRY_MEASURED)
    Serial.println(F("WARNING: geometry/ToF-offset values are placeholders - measure them (see top of sketch)."));
  if (!calibrate()) { errorBlink(); return; }
  Serial.println(F("Starting in 1.5 s ..."));
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    digitalWrite(PIN_LED, ((millis() - t0) / 100) & 1);
    if (userStopRequested()) { Serial.println(F("Start cancelled.")); return; }
    delay(5);
  }
  resetControl();
  g_avState = AV_FOLLOW;
  g_speedCap = 255.0f;
  g_resumeCapUntil = 0;
  g_testDetour = false;
  tofResetCounters();
  g_runStart = millis();
  g_rampStart = g_runStart;
  g_state = ST_RUN;
  digitalWrite(PIN_LED, HIGH);
  Serial.println(F("RUNNING (press BOOT or send 's' to stop)"));
}

static void motorTest() {
  Serial.println(F("Motor test: LEFT fwd/back, RIGHT fwd/back"));
  setMotors(120, 0);  delay(700); motorsStop(true); delay(300);
  setMotors(-120, 0); delay(700); motorsStop(true); delay(300);
  setMotors(0, 120);  delay(700); motorsStop(true); delay(300);
  setMotors(0, -120); delay(700); motorsStop(true);
}

static void printDistance() {
  Serial.println(F("ToF distance (3 s):"));
  for (int i = 0; i < 15; i++) {
    tofRefresh();
    Serial.printf("  %u mm  fresh=%d close=%u stop=%u clear=%u emg=%u\n",
                  (unsigned)g_dist, (int)tofFresh(), g_closeCnt, g_stopCnt, g_clearCnt, g_emgCnt);
    delay(200);
  }
}

static void printIR() {
  Serial.println(F("IR sensors (3 s): raw bits IR1..IR7 | active (line) bits | pos"));
  for (int i = 0; i < 15; i++) {
    Serial.print(F("  raw="));
    for (int k = 0; k < 7; k++) Serial.print(digitalRead(IR_PIN[k]) == HIGH ? '1' : '0');
    readSensors();
    Serial.print(F("  active="));
    for (int k = 0; k < 7; k++) Serial.print((g_mask >> k) & 1);
    Serial.printf("  cnt=%u pos=%.2f%s\n", g_cnt, g_pos, g_calibrated ? "" : "  (not calibrated: 'active' is meaningless)");
    delay(200);
  }
}

static void printHelp() {
  Serial.println(F("Commands: s start/stop | c calibrate | d debug | t motor test | v ToF distance"));
  Serial.println(F("          i IR sensors | g geometry | r/l turn test | f shift test | p pass test"));
  Serial.println(F("          o detour test | h help"));
}

static void handleCommand(char c) {
  switch (c) {
    case 's': case 'S':
      if (g_state == ST_RUN) haltRobot("stopped by user");
      else startSequence();
      break;
    case 'c': case 'C': if (g_state != ST_RUN) calibrate(); break;
    case 'd': case 'D': g_debug = !g_debug; Serial.println(g_debug ? F("debug ON") : F("debug OFF")); break;
    case 't': case 'T': if (g_state != ST_RUN) motorTest(); break;
    case 'v': case 'V': if (g_state != ST_RUN) printDistance(); break;
    case 'i': case 'I': if (g_state != ST_RUN) printIR(); break;
    case 'g': case 'G': printGeometry(); break;
    case 'r': case 'R':
      if (g_state != ST_RUN) { Serial.println(F("Turn RIGHT test")); timedRun(AV_TURN_SPEED, -AV_TURN_SPEED, TURN_RIGHT_MS); }
      break;
    case 'l': case 'L':
      if (g_state != ST_RUN) { Serial.println(F("Turn LEFT test")); timedRun(-AV_TURN_SPEED, AV_TURN_SPEED, TURN_LEFT_MS); }
      break;
    case 'f': case 'F':
      if (g_state != ST_RUN) { Serial.println(F("Shift test (RIGHT_SHIFT_MS)")); timedRun(AV_DRIVE_SPEED, AV_DRIVE_SPEED, g_rightShiftMs); }
      break;
    case 'p': case 'P':
      if (g_state != ST_RUN) { Serial.println(F("Pass test (FORWARD_PASS_MS)")); timedRun(AV_DRIVE_SPEED, AV_DRIVE_SPEED, g_forwardPassMs); }
      break;
    case 'o': case 'O':
      if (g_state != ST_RUN) {
        if (!g_calibrated) { Serial.println(F("Calibrate first (c).")); break; }
        Serial.println(F("Detour test: starting at TURN_RIGHT (robot must be at the stop position)"));
        resetControl();
        g_runStart = millis();
        g_rampStart = g_runStart;
        g_resumeCapUntil = 0;
        g_testDetour = true;
        g_state = ST_RUN;
        digitalWrite(PIN_LED, HIGH);
        avGoto(AV_TURN_RIGHT);
      }
      break;
    case 'h': case 'H': case '?': printHelp(); break;
    default: break;
  }
}

// ============================== SETUP / LOOP ================================
void setup() {
  Serial.begin(115200);
  delay(200);

  for (uint8_t i = 0; i < 7; i++) pinMode(IR_PIN[i], INPUT);

  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  PWM_ATTACH(PIN_PWMA, CH_A);
  PWM_ATTACH(PIN_PWMB, CH_B);
  motorsStop(false);

  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  computeGeometry();            // must run BEFORE the ToF task starts (task reads the thresholds)

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  for (int i = 0; i < 5 && !g_tofOk; i++) {
    g_tofOk = lox.begin();
    if (!g_tofOk) delay(100);
  }
  if (g_tofOk) {
    t_time = millis();
    g_distTime = t_time;
    xTaskCreatePinnedToCore(tofTask, "tof", 6144, NULL, 1, NULL, 0);
    Serial.println(F("VL53L0X OK"));
  } else {
    Serial.println(F("WARNING: VL53L0X not found - obstacle avoidance DISABLED"));
  }

  printGeometry();
  Serial.println(F("\nBPUT T12 robot ready. Put IR4 on the line, press BOOT or send 's'."));
  printHelp();
}

void loop() {
  char c = 0;
  if (buttonEvent()) c = 's';
  else if (Serial.available()) c = Serial.read();
  if (c) handleCommand(c);

  if (g_state == ST_RUN) {
    runStep();
  } else {
    digitalWrite(PIN_LED, (millis() / 500) & 1);   // slow blink = idle
    delay(5);
  }
}
