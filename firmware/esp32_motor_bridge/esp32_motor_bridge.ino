/*
 * Aegis rescue robot — ESP32 M3508 drive bridge
 * =============================================
 *
 * Derived from `Postion_Velocity_control_robomaster_M3508_motor-main/main.ino`
 * (four M3508 motors over MCP2515 CAN, PS4 pad, velocity + incremental position
 * control). That sketch only *printed* over USB serial; this version also
 * accepts dashboard commands from the Raspberry Pi on the same link and streams
 * telemetry back, so the dashboard can drive the robot.
 *
 * Serial protocol (newline-delimited ASCII, 115200 baud)
 * ------------------------------------------------------
 * Host -> ESP32
 *   VEL,<left_rpm>,<right_rpm>  side wheel RPM, positive = forward. Implies
 *                               VELOCITY mode. Unacknowledged (sent at 20 Hz).
 *   CUR,<left_mA>,<right_mA>    direct C620 current, positive = forward.
 *                               Clamped to +/-10000 mA. Implies CURRENT mode.
 *                               Unacknowledged (sent at 20 Hz).
 *   POS,<dx_mm>,<dtheta_mrad>   relative body move. The first POS after any
 *                               other mode holds the current position, then
 *                               adds the requested delta. Implies POSITION
 *                               mode. Send PING to keep the link alive.
 *   STOP                        zero RPM/current and lock position targets,
 *                               returns the host to VELOCITY mode
 *   ESTOP                       latch emergency stop, cut drive output
 *   RESET                       clear the latch and hold zero
 *   PING                        keep-alive, replies OK,pong. Does not change
 *                               the active drive mode or position target.
 *
 * ESP32 -> Host
 *   TEL,<ms>,<estop>,<cmd_l>,<cmd_r>,<rpm_l>,<rpm_r>,<ticks_l>,<ticks_r>
 *   DRV,<T|P>,<cmd_l>,<cmd_r>,<tgt_l>,<tgt_r>,<err_l>,<err_r>
 *       Emitted at 5 Hz while a host CURRENT or POSITION session is active.
 *       `cmd_*` is mA in CURRENT mode; `tgt_*`/`err_*` are ticks in POSITION.
 *   OK,<message> / ERR,<message>
 *
 * `ticks_*` and `rpm_*` are side-normalised (positive = forward); left averages
 * LF/LB and right averages RF/RB with the right side inverted.
 *
 * Safety
 * ------
 * - If no host command arrives for HOST_TIMEOUT_MS all host outputs (RPM,
 *   current and position PID) are forced to zero. After HOST_LINK_IDLE_MS the
 *   PS4 pad takes over again (so the robot can still be driven if the Pi or the
 *   USB cable dies).
 * - CURRENT commands are clamped in firmware and the POSITION PID output is
 *   limited to +/-HOST_MAX_CURRENT_RAW (10 A) in host mode.
 * - ESTOP latches until a host RESET (or a power cycle) and overrides both the
 *   host and the pad.
 * - The Pi bridge mirrors this with its own command hold window.
 *
 * Libraries: mcp_can, PS4Controller (same as the original sketch).
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <PS4Controller.h>

// ==========================
// MCP2515 PIN & CAN SETUP
// ==========================
#define CAN_CS 5
#define CAN_INT 4
MCP_CAN CAN(CAN_CS);

// ==========================
// ROBOT GEOMETRY CONFIG
// ==========================
const float WHEEL_RADIUS = 0.05f;        // meters (5 cm)
const float TRACK_WIDTH = 0.28f;         // meters (distance between left/right wheels)
const float GEAR_RATIO = 1.0f;           // Set to your actual gear ratio
const int TICKS_PER_MOTOR_REV = 8192;    // C620 encoder resolution
const float TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;
// Measured by hand (one wheel turn each): the sides differ, so position
// targets must use per-side tick counts or moves arc.
const float TICKS_PER_WHEEL_REV_LEFT = 36902.0f;
const float TICKS_PER_WHEEL_REV_RIGHT = 41564.0f;
// Per-motor minimum start RPM (LF, LB, RF, RB): motors are not identical, so
// a shared floor drops out the weakest ones first. Raise an entry if its
// motor still stalls while the others turn. Zero disables flooring.
const int16_t MOTOR_MIN_RPM[4] = {500, 500, 500, 500};

// ==========================
// PID PARAMETERS
// ==========================
float Kp = 0.4f;
float Ki = 0.0003f;
float Kd = 0.015f;
const int16_t MAX_RPM = 1500;
const float INTEGRAL_LIMIT = 1e6f;
const int32_t POSITION_DEADBAND = 80;    // ticks (stop when within this range)

// ==========================
// CONTROL TIMING
// ==========================
unsigned long lastSend = 0;
unsigned long lastControl = 0;
unsigned long lastTelemetry = 0;
const unsigned long sendInterval = 20;         // 50 Hz control loop
const unsigned long TELEMETRY_INTERVAL = 50;   // 20 Hz telemetry to the Pi

// ==========================
// HOST LINK (Raspberry Pi)
// ==========================
const unsigned long HOST_TIMEOUT_MS = 400;      // zero the wheels after this silence
const unsigned long HOST_LINK_IDLE_MS = 2000;   // hand control back to the PS4 pad
const int16_t HOST_MAX_RPM = MAX_RPM;
const unsigned long DRIVE_TELEMETRY_INTERVAL = 200;  // 5 Hz DRV status line

// C620 current command range: 16384 raw = 20 A, so 8192 raw = 10 A.
const float C620_MAX_AMPS = 20.0f;
const int16_t C620_MAX_RAW = 16384;
const int16_t HOST_MAX_CURRENT_MA = 10000;
const int16_t HOST_MAX_CURRENT_RAW = 8192;
// Position-mode PID output cap in RPM: without this the PID converts the full
// tick error directly (Kp 0.4 x 7823 ticks = ~3100 RPM) and slams the motors
// into the target, overshooting and stressing the mounts. 350 RPM keeps
// position/coordinate moves firm but controlled.
const int16_t HOST_MAX_POS_RPM = 350;
const float CURRENT_MA_TO_RAW = (float)C620_MAX_RAW / (C620_MAX_AMPS * 1000.0f);
const long HOST_MAX_POS_DX_MM = 500;          // max 0.5 m per POS line
const long HOST_MAX_POS_DTHETA_MRAD = 1571;   // max ~0.5 turn per POS line

bool hostActive = false;
bool estopLatched = false;
unsigned long lastHostCmdMs = 0;
unsigned long lastDriveTelemetry = 0;
int16_t hostLeftRpm = 0;
int16_t hostRightRpm = 0;
int16_t hostLeftCurrentMa = 0;
int16_t hostRightCurrentMa = 0;

// Single-motor test mode (MTR command): drive one motor at a time to find
// each motor's start threshold. Cleared by any other host command.
bool singleMotorTest = false;
int testMotorIdx = -1;
int16_t testMotorRpm = 0;

// ==========================
// MOTOR STRUCTURE
// ==========================
struct Motor {
  // Encoder tracking
  uint16_t last_raw_angle = 0;
  int64_t total_ticks = 0;
  int64_t target_ticks = 0;
  int16_t latest_rpm_feedback = 0;

  // PID state
  float integral = 0.0f;
  int64_t last_error = 0;
  int64_t last_position = 0;

  // Status
  bool position_reached = true;
};

// Motor indices: 0=LF, 1=LB, 2=RF, 3=RB
Motor motors[4];

// ==========================
// CAN IDs
// ==========================
const int sendID = 0x200;
const int feedbackIDs[4] = {0x202, 0x204, 0x203, 0x201}; // LF, LB, RF, RB

// ==========================
// CONTROL MODE
// ==========================
enum ControlMode {
  MODE_VELOCITY,   // Direct joystick / host RPM control
  MODE_POSITION    // Position control with incremental moves (pad only)
};
ControlMode controlMode = MODE_VELOCITY;

// Host (Raspberry Pi) drive mode is independent of the pad's ControlMode so
// torque/position commands cannot be misinterpreted by the joystick logic.
enum HostDriveMode {
  HOST_VELOCITY,
  HOST_CURRENT,
  HOST_POSITION
};
HostDriveMode hostDriveMode = HOST_VELOCITY;

// ==========================
// PS4 VARIABLES
// ==========================
int xData = 0, yData = 0;
int X = 0, Y = 0;

// ==========================
// UTILITY FUNCTIONS
// ==========================
inline float wheelCircumference() {
  return 2.0f * M_PI * WHEEL_RADIUS;
}

inline float metersToTicks(float meters) {
  float revs = meters / wheelCircumference();
  return revs * TICKS_PER_WHEEL_REV;
}

inline float metersToTicksSide(float meters, bool leftSide) {
  float revs = meters / wheelCircumference();
  return revs * (leftSide ? TICKS_PER_WHEEL_REV_LEFT : TICKS_PER_WHEEL_REV_RIGHT);
}

// Floor one motor's RPM command to its start threshold (sign preserved, zero
// stays zero) so all four motors break friction together.
inline int16_t applyMotorMinRpm(int index, int16_t rpm) {
  int16_t floorRpm = MOTOR_MIN_RPM[index];
  if (rpm == 0 || floorRpm <= 0) return rpm;
  if (rpm > 0) return rpm >= floorRpm ? rpm : floorRpm;
  return rpm <= -floorRpm ? rpm : (int16_t)-floorRpm;
}

inline float ticksToMeters(int64_t ticks) {
  return (float)ticks / TICKS_PER_WHEEL_REV * wheelCircumference();
}

// Handle encoder wraparound (0..8191)
int wrapDiff(int new_raw, int old_raw) {
  int diff = new_raw - old_raw;
  if (diff > (TICKS_PER_MOTOR_REV / 2)) diff -= TICKS_PER_MOTOR_REV;
  else if (diff < -(TICKS_PER_MOTOR_REV / 2)) diff += TICKS_PER_MOTOR_REV;
  return diff;
}

// ==========================
// ENCODER FEEDBACK HANDLER
// ==========================
void handleCANFeedback(long unsigned int rxId, byte rxBuf[]) {
  // Find which motor this feedback is for
  int motorIdx = -1;
  for (int i = 0; i < 4; i++) {
    if (rxId == feedbackIDs[i]) {
      motorIdx = i;
      break;
    }
  }
  if (motorIdx == -1) return;

  Motor &m = motors[motorIdx];

  // Parse encoder angle (bytes 0-1, big endian, 0..8191)
  uint16_t raw_angle = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
  int diff = wrapDiff((int)raw_angle, (int)m.last_raw_angle);
  m.total_ticks += diff;
  m.last_raw_angle = raw_angle;

  // Parse RPM feedback (bytes 2-3, signed)
  int16_t rpm_fb = (int16_t)(((uint16_t)rxBuf[2] << 8) | rxBuf[3]);
  m.latest_rpm_feedback = rpm_fb;

  // Check if position reached
  int64_t error = m.target_ticks - m.total_ticks;
  m.position_reached = (llabs(error) < POSITION_DEADBAND);
}

// ==========================
// POSITION PID CONTROLLER
// ==========================
int16_t computePIDandGetRPM(Motor &m, int16_t max_output = MAX_RPM) {
  int64_t error = m.target_ticks - m.total_ticks;

  // Deadband - stop when close enough
  if (llabs(error) < POSITION_DEADBAND) {
    m.integral = 0;
    m.last_error = error;
    return 0;
  }

  // Integral with conditional integration (only integrate when error is reasonable)
  if (llabs(error) < 5000) {
    m.integral += (float)error * (sendInterval / 1000.0f);
    m.integral = constrain(m.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  // Derivative on measurement (avoids derivative kick)
  int64_t position_change = m.total_ticks - m.last_position;
  float derivative = -(float)position_change / (sendInterval / 1000.0f);
  m.last_position = m.total_ticks;

  // PID calculation
  float pid_out = Kp * (float)error + Ki * m.integral + Kd * derivative;

  // Convert to the output unit (RPM for the pad, C620 raw for host torque)
  // and clamp to the caller's limit.
  int16_t rpm_cmd = (int16_t)round(pid_out);
  rpm_cmd = constrain(rpm_cmd, (int16_t)-max_output, max_output);

  m.last_error = error;
  return rpm_cmd;
}

// ==========================
// DIFFERENTIAL DRIVE KINEMATICS
// ==========================
void moveByBodyDelta(float dx_meters, float dtheta_radians) {
  // Calculate left and right wheel displacements
  float s_left = dx_meters - dtheta_radians * (TRACK_WIDTH / 2.0f);
  float s_right = dx_meters + dtheta_radians * (TRACK_WIDTH / 2.0f);

  int32_t ticks_left = (int32_t)round(metersToTicksSide(s_left, true));
  int32_t ticks_right = (int32_t)round(metersToTicksSide(s_right, false));

  // Left side positive = forward, right side negative = forward
  motors[0].target_ticks += ticks_left;    // LF
  motors[1].target_ticks += ticks_left;    // LB
  motors[2].target_ticks -= ticks_right;   // RF
  motors[3].target_ticks -= ticks_right;   // RB

  for (int i = 0; i < 4; i++) {
    motors[i].position_reached = false;
    motors[i].integral = 0;  // Reset integral on new move
  }

  Serial.printf("# Move cmd: dx=%.3f m, dtheta=%.3f rad (L=%ld R=%ld ticks)\n",
                dx_meters, dtheta_radians, (long)ticks_left, (long)ticks_right);
}

void stopAllPosition() {
  for (int i = 0; i < 4; i++) {
    motors[i].target_ticks = motors[i].total_ticks;
    motors[i].integral = 0;
    motors[i].last_error = 0;
    motors[i].position_reached = true;
  }
}

// Hold the current encoder position. Called on the first POS after another
// mode, so `POS,0,0` acts as a hold command and later POS lines add to it.
void enterHostPositionMode() {
  for (int i = 0; i < 4; i++) {
    motors[i].target_ticks = motors[i].total_ticks;
    motors[i].integral = 0;
    motors[i].last_error = 0;
    motors[i].last_position = motors[i].total_ticks;
    motors[i].position_reached = true;
  }
}

void zeroHostCommands() {
  hostLeftRpm = 0;
  hostRightRpm = 0;
  hostLeftCurrentMa = 0;
  hostRightCurrentMa = 0;
}

// ==========================
// VELOCITY MODE FUNCTIONS
// ==========================
int16_t LF_vel = 0, LB_vel = 0, RF_vel = 0, RB_vel = 0;

// Side-normalised velocity: both left motors forward for +left, both right
// motors reversed for +right (matches the wheel mounting on this chassis).
void applySideVelocity(int16_t left_rpm, int16_t right_rpm) {
  LF_vel = left_rpm;
  LB_vel = left_rpm;
  RF_vel = -right_rpm;
  RB_vel = -right_rpm;
}

void stopAll_vel() {
  LF_vel = LB_vel = RF_vel = RB_vel = 0;
}

void axis_logic_velocity(int X, int Y) {
  int deadzone = 20;

  if (abs(X) < deadzone && abs(Y) < deadzone) {
    stopAll_vel();
    return;
  }

  if (abs(Y) >= abs(X)) {
    // Drive: both sides share the same side-normalised RPM.
    applySideVelocity(Y, Y);
  } else {
    // Pure spin: +X turns right (clockwise), -X turns left (counter-clockwise).
    applySideVelocity(X, -X);
  }
}

// ==========================
// POSITION MODE JOYSTICK LOGIC
// ==========================
void axis_logic_position(int X, int Y) {
  static unsigned long lastMoveTime = 0;
  const unsigned long moveInterval = 200;  // Min time between moves (ms)
  int deadzone = 30;

  // Check if all motors reached their targets
  bool allReached = true;
  for (int i = 0; i < 4; i++) {
    if (!motors[i].position_reached) {
      allReached = false;
      break;
    }
  }

  // Only accept new commands if motors are done AND enough time passed
  if (!allReached || (millis() - lastMoveTime < moveInterval)) {
    return;
  }

  if (abs(X) < deadzone && abs(Y) < deadzone) {
    return;  // No movement
  }

  // Convert joystick to small incremental moves
  float scale = 0.02f;  // meters per command (tune this!)
  float dx = (Y / 1500.0f) * scale;      // Forward/backward
  float dtheta = (X / 1500.0f) * 0.3f;   // Turn (radians, tune this!)

  moveByBodyDelta(dx, dtheta);
  lastMoveTime = millis();
}

// ==========================
// CAN SEND FUNCTION
// ==========================
void sendMotorData(int16_t rpm_LF, int16_t rpm_LB, int16_t rpm_RF, int16_t rpm_RB) {
  byte data[8];

  // 0x200 expects motors in the order 0x201, 0x202, 0x203, 0x204
  // i.e. RB, LF, RF, LB
  data[0] = (rpm_RB >> 8) & 0xFF;
  data[1] = rpm_RB & 0xFF;
  data[2] = (rpm_LF >> 8) & 0xFF;
  data[3] = rpm_LF & 0xFF;
  data[4] = (rpm_RF >> 8) & 0xFF;
  data[5] = rpm_RF & 0xFF;
  data[6] = (rpm_LB >> 8) & 0xFF;
  data[7] = rpm_LB & 0xFF;

  CAN.sendMsgBuf(sendID, 0, 8, data);
}

// ==========================
// HOST SERIAL PROTOCOL
// ==========================
char commandBuffer[80];
uint8_t commandLength = 0;

int16_t clampRpm(long rpm) {
  return (int16_t)constrain(rpm, (long)-HOST_MAX_RPM, (long)HOST_MAX_RPM);
}

int16_t clampCurrentMa(long ma) {
  return (int16_t)constrain(ma, (long)-HOST_MAX_CURRENT_MA, (long)HOST_MAX_CURRENT_MA);
}

int16_t currentMaToRaw(int16_t ma) {
  long raw = lroundf((float)ma * CURRENT_MA_TO_RAW);
  return (int16_t)constrain(raw, (long)-C620_MAX_RAW, (long)C620_MAX_RAW);
}

// Same side convention as applySideVelocity: positive current drives both
// left motors forward and both right motors backward (right side inverted).
void applySideCurrent(int16_t left_ma, int16_t right_ma) {
  int16_t left_raw = currentMaToRaw(left_ma);
  int16_t right_raw = currentMaToRaw(right_ma);
  LF_vel = left_raw;
  LB_vel = left_raw;
  RF_vel = -right_raw;
  RB_vel = -right_raw;
}

void markHostCommand() {
  hostActive = true;
  lastHostCmdMs = millis();
}

void handleHostCommand(char *line) {
  // Trim trailing whitespace
  size_t length = strlen(line);
  while (length > 0 && isspace((unsigned char)line[length - 1])) {
    line[--length] = '\0';
  }
  if (length == 0) {
    return;
  }

  // Any non-test command exits single-motor test mode.
  if (strncmp(line, "MTR,", 4) != 0 && strcmp(line, "STAT") != 0) {
    singleMotorTest = false;
    testMotorIdx = -1;
    testMotorRpm = 0;
  }

  if (strcmp(line, "PING") == 0) {
    markHostCommand();
    Serial.println("OK,pong");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    markHostCommand();
    zeroHostCommands();
    stopAll_vel();
    stopAllPosition();
    hostDriveMode = HOST_VELOCITY;
    Serial.println("OK,stop");
    return;
  }

  if (strcmp(line, "ESTOP") == 0) {
    markHostCommand();
    estopLatched = true;
    zeroHostCommands();
    stopAll_vel();
    stopAllPosition();
    hostDriveMode = HOST_VELOCITY;
    Serial.println("OK,estop");
    return;
  }

  if (strcmp(line, "RESET") == 0) {
    markHostCommand();
    estopLatched = false;
    zeroHostCommands();
    stopAll_vel();
    stopAllPosition();
    hostDriveMode = HOST_VELOCITY;
    Serial.println("OK,reset");
    return;
  }

  if (strncmp(line, "MTR,", 4) == 0) {
    char *idxText = line + 4;
    char *comma = strchr(idxText, ',');
    if (comma == NULL) {
      Serial.println("ERR,mtr-format");
      return;
    }
    *comma = '\0';
    long idx = strtol(idxText, NULL, 10);
    long rpm = strtol(comma + 1, NULL, 10);
    if (idx < 0 || idx > 3) {
      Serial.println("ERR,mtr-idx");
      return;
    }
    singleMotorTest = (rpm != 0);
    testMotorIdx = (int)idx;
    testMotorRpm = (int16_t)constrain(rpm, (long)-HOST_MAX_RPM, (long)HOST_MAX_RPM);
    hostDriveMode = HOST_VELOCITY;
    markHostCommand();
    Serial.println("OK,mtr");
    return;
  }

  if (strcmp(line, "STAT") == 0) {
    markHostCommand();
    const char* names[] = {"LF", "LB", "RF", "RB"};
    for (int i = 0; i < 4; i++) {
      Serial.printf("# MTR %s pos=%lld rpm=%d\n",
                    names[i],
                    motors[i].total_ticks,
                    motors[i].latest_rpm_feedback);
    }
    Serial.println("OK,stat");
    return;
  }

  if (strncmp(line, "VEL,", 4) == 0) {
    char *leftText = line + 4;
    char *comma = strchr(leftText, ',');
    if (comma == NULL) {
      Serial.println("ERR,vel-format");
      return;
    }
    *comma = '\0';
    char *rightText = comma + 1;

    long left = strtol(leftText, NULL, 10);
    long right = strtol(rightText, NULL, 10);
    if (hostDriveMode == HOST_POSITION) {
      stopAllPosition();
    }
    hostDriveMode = HOST_VELOCITY;
    hostLeftCurrentMa = 0;
    hostRightCurrentMa = 0;
    hostLeftRpm = clampRpm(left);
    hostRightRpm = clampRpm(right);
    controlMode = MODE_VELOCITY;  // host drive is velocity based
    markHostCommand();
    return;  // VEL is unacknowledged: it arrives at 20 Hz
  }

  if (strncmp(line, "CUR,", 4) == 0) {
    char *leftText = line + 4;
    char *comma = strchr(leftText, ',');
    if (comma == NULL) {
      Serial.println("ERR,cur-format");
      return;
    }
    *comma = '\0';
    char *rightText = comma + 1;

    long left = strtol(leftText, NULL, 10);
    long right = strtol(rightText, NULL, 10);
    if (hostDriveMode == HOST_POSITION) {
      stopAllPosition();
    }
    hostDriveMode = HOST_CURRENT;
    hostLeftRpm = 0;
    hostRightRpm = 0;
    hostLeftCurrentMa = clampCurrentMa(left);
    hostRightCurrentMa = clampCurrentMa(right);
    markHostCommand();
    return;  // CUR is unacknowledged: it arrives at 20 Hz
  }

  if (strncmp(line, "POS,", 4) == 0) {
    char *dxText = line + 4;
    char *firstComma = strchr(dxText, ',');
    if (firstComma == NULL) {
      Serial.println("ERR,pos-format");
      return;
    }
    *firstComma = '\0';
    char *dthetaText = firstComma + 1;
    char *secondComma = strchr(dthetaText, ',');
    if (secondComma != NULL) {
      *secondComma = '\0';
    }

    long dx_mm = strtol(dxText, NULL, 10);
    long dtheta_mrad = strtol(dthetaText, NULL, 10);
    dx_mm = constrain(dx_mm, -HOST_MAX_POS_DX_MM, HOST_MAX_POS_DX_MM);
    dtheta_mrad = constrain(dtheta_mrad, -HOST_MAX_POS_DTHETA_MRAD, HOST_MAX_POS_DTHETA_MRAD);

    if (hostDriveMode != HOST_POSITION) {
      enterHostPositionMode();
      hostDriveMode = HOST_POSITION;
    }
    hostLeftRpm = 0;
    hostRightRpm = 0;
    hostLeftCurrentMa = 0;
    hostRightCurrentMa = 0;

    if (dx_mm != 0 || dtheta_mrad != 0) {
      moveByBodyDelta((float)dx_mm / 1000.0f, (float)dtheta_mrad / 1000.0f);
    }
    markHostCommand();
    return;  // POS is unacknowledged; stream PING for keep-alive
  }

  Serial.println("ERR,unknown");
}

// Non-blocking line reader; unknown/garbage lines are reported, never fatal.
void pollHostSerial() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleHostCommand(commandBuffer);
        commandLength = 0;
      }
      continue;
    }
    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      // Overlong frame: drop it rather than replaying garbage.
      commandLength = 0;
      Serial.println("ERR,overflow");
    }
  }
}

bool hostLinkStale() {
  return hostActive && (millis() - lastHostCmdMs > HOST_TIMEOUT_MS);
}

// Zero the wheels when the host stops talking, and hand control back to the
// pad if the link stays silent (Pi reboot, USB unplug).
void serviceHostLink() {
  if (!hostActive) {
    return;
  }
  unsigned long now = millis();
  if (now - lastHostCmdMs > HOST_TIMEOUT_MS) {
    if (hostDriveMode == HOST_POSITION) {
      // Abort the move instead of resuming it if PING returns later.
      stopAllPosition();
    }
    zeroHostCommands();
  }
  if (now - lastHostCmdMs > HOST_LINK_IDLE_MS) {
    hostActive = false;
    zeroHostCommands();
    stopAll_vel();
    stopAllPosition();
    hostDriveMode = HOST_VELOCITY;
    controlMode = MODE_VELOCITY;
    Serial.println("# host link idle; PS4 control resumed");
  }
}

void sendTelemetry() {
  int32_t ticksLeft = (int32_t)((motors[0].total_ticks + motors[1].total_ticks) / 2);
  int32_t ticksRight = (int32_t)(-(motors[2].total_ticks + motors[3].total_ticks) / 2);
  int16_t rpmLeft = (int16_t)((motors[0].latest_rpm_feedback + motors[1].latest_rpm_feedback) / 2);
  int16_t rpmRight = (int16_t)(-(motors[2].latest_rpm_feedback + motors[3].latest_rpm_feedback) / 2);

  Serial.printf("TEL,%lu,%d,%d,%d,%d,%d,%ld,%ld\n",
                millis(),
                estopLatched ? 1 : 0,
                hostLeftRpm,
                hostRightRpm,
                rpmLeft,
                rpmRight,
                (long)ticksLeft,
                (long)ticksRight);
}

// Extra status line for CURRENT/POSITION sessions. The existing parser ignores
// DRV frames, so older bridges keep working with the 9-field TEL format.
void sendDriveTelemetry() {
  if (!hostActive || hostDriveMode == HOST_VELOCITY) {
    return;
  }

  char modeChar = (hostDriveMode == HOST_CURRENT) ? 'T' : 'P';
  int16_t cmdLeft = hostLeftCurrentMa;
  int16_t cmdRight = hostRightCurrentMa;
  int32_t targetLeft = 0;
  int32_t targetRight = 0;
  int32_t errorLeft = 0;
  int32_t errorRight = 0;

  if (hostDriveMode == HOST_POSITION) {
    int64_t leftTarget = (motors[0].target_ticks + motors[1].target_ticks) / 2;
    int64_t rightTarget = (motors[2].target_ticks + motors[3].target_ticks) / 2;
    int64_t leftError = ((motors[0].target_ticks - motors[0].total_ticks) +
                         (motors[1].target_ticks - motors[1].total_ticks)) / 2;
    int64_t rightError = ((motors[2].target_ticks - motors[2].total_ticks) +
                          (motors[3].target_ticks - motors[3].total_ticks)) / 2;
    targetLeft = (int32_t)leftTarget;
    targetRight = (int32_t)-rightTarget;
    errorLeft = (int32_t)leftError;
    errorRight = (int32_t)-rightError;
    cmdLeft = 0;
    cmdRight = 0;
  }

  Serial.printf("DRV,%c,%d,%d,%ld,%ld,%ld,%ld\n",
                modeChar,
                cmdLeft,
                cmdRight,
                (long)targetLeft,
                (long)targetRight,
                (long)errorLeft,
                (long)errorRight);
}

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("# === Aegis ESP32 drive bridge starting ===");

  // Initialize CAN
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("# CAN initialized");
  } else {
    Serial.println("# CAN init failed - check MCP2515 wiring/clock");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);

  // Initialize PS4
  PS4.begin("18:67:B0:51:0E:1D");
  Serial.println("# PS4 controller ready");

  // Initialize motor states
  for (int i = 0; i < 4; i++) {
    motors[i].total_ticks = 0;
    motors[i].target_ticks = 0;
    motors[i].integral = 0;
    motors[i].last_error = 0;
    motors[i].position_reached = true;
  }

  Serial.println("# Host control: VEL,<left_rpm>,<right_rpm> | CUR,<left_mA>,<right_mA> | POS,<dx_mm>,<dtheta_mrad>");
  Serial.println("#               STOP | ESTOP | RESET | PING (keep-alive; stream it in POSITION)");
  Serial.println("# Current clamp: +/-10000 mA; position PID limit: +/-8192 raw (10 A)");
  Serial.println("# Pad fallback resumes after 2 s without host commands");

  lastControl = millis();
}

// ==========================
// MAIN LOOP
// ==========================
void loop() {
  pollHostSerial();
  serviceHostLink();

  // Read CAN feedback
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    byte len;
    byte rxBuf[8];
    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      handleCANFeedback(rxId, rxBuf);
    }
  }

  // Read PS4 controller
  if (PS4.isConnected()) {
    xData = PS4.RStickX();
    yData = PS4.RStickY();
    X = map(xData, -128, 127, -1500, 1500);
    Y = map(yData, -128, 127, -1500, 1500);

    // Toggle mode with L1 (ignored while the host is driving)
    static bool l1WasPressed = false;
    if (PS4.L1() && !hostActive) {
      if (!l1WasPressed) {
        controlMode = (controlMode == MODE_VELOCITY) ? MODE_POSITION : MODE_VELOCITY;
        Serial.printf("# Mode: %s\n", (controlMode == MODE_VELOCITY) ? "VELOCITY" : "POSITION");

        // When switching to position mode, set current position as target
        if (controlMode == MODE_POSITION) {
          for (int i = 0; i < 4; i++) {
            motors[i].target_ticks = motors[i].total_ticks;
            motors[i].integral = 0;
            motors[i].position_reached = true;
          }
        }

        l1WasPressed = true;
      }
    } else {
      l1WasPressed = false;
    }

    // Stop button (Circle) - only in position mode and only without a host
    if (PS4.Circle() && controlMode == MODE_POSITION && !hostActive) {
      stopAllPosition();
    }

    // Status button (Triangle)
    if (PS4.Triangle()) {
      Serial.println("# === Motor Status ===");
      for (int i = 0; i < 4; i++) {
        const char* names[] = {"LF", "LB", "RF", "RB"};
        Serial.printf("# %s: pos=%lld tgt=%lld err=%lld rpm=%d %s\n",
                      names[i],
                      motors[i].total_ticks,
                      motors[i].target_ticks,
                      motors[i].target_ticks - motors[i].total_ticks,
                      motors[i].latest_rpm_feedback,
                      motors[i].position_reached ? "ok" : "...");
      }
    }
  } else {
    // Pad disconnected: never reuse the last stick values after host idle.
    X = 0;
    Y = 0;
  }

  // Control loop
  unsigned long now = millis();
  if (now - lastControl >= sendInterval) {
    lastControl = now;

    int16_t rpm_cmds[4] = {0, 0, 0, 0};

    if (estopLatched) {
      // Latched stop: the PID and any joystick input are ignored.
      rpm_cmds[0] = rpm_cmds[1] = rpm_cmds[2] = rpm_cmds[3] = 0;

    } else if (hostActive) {
      if (hostLinkStale()) {
        // Link lost: force zero even in POSITION, where the PID would
        // otherwise keep driving toward the last target.
        rpm_cmds[0] = rpm_cmds[1] = rpm_cmds[2] = rpm_cmds[3] = 0;

      } else if (singleMotorTest && testMotorIdx >= 0) {
        // Single-motor test: only the selected motor gets its setpoint.
        for (int i = 0; i < 4; i++) rpm_cmds[i] = 0;
        rpm_cmds[testMotorIdx] = testMotorRpm;

      } else if (hostDriveMode == HOST_CURRENT) {
        // Dashboard torque drive: side current straight from the Pi bridge.
        applySideCurrent(hostLeftCurrentMa, hostRightCurrentMa);
        rpm_cmds[0] = LF_vel;
        rpm_cmds[1] = LB_vel;
        rpm_cmds[2] = RF_vel;
        rpm_cmds[3] = RB_vel;

      } else if (hostDriveMode == HOST_POSITION) {
        // Host position hold/move, RPM capped so moves stay controlled.
        for (int i = 0; i < 4; i++) {
          rpm_cmds[i] = applyMotorMinRpm(i, computePIDandGetRPM(motors[i], HOST_MAX_POS_RPM));
        }

      } else {
        // Dashboard velocity drive: side RPM straight from the Pi bridge,
        // floored per motor so weak motors don't stall out.
        applySideVelocity(hostLeftRpm, hostRightRpm);
        rpm_cmds[0] = applyMotorMinRpm(0, LF_vel);
        rpm_cmds[1] = applyMotorMinRpm(1, LB_vel);
        rpm_cmds[2] = applyMotorMinRpm(2, RF_vel);
        rpm_cmds[3] = applyMotorMinRpm(3, RB_vel);
      }

    } else if (controlMode == MODE_VELOCITY) {
      axis_logic_velocity(X, Y);
      rpm_cmds[0] = LF_vel;
      rpm_cmds[1] = LB_vel;
      rpm_cmds[2] = RF_vel;
      rpm_cmds[3] = RB_vel;

    } else {
      axis_logic_position(X, Y);

      for (int i = 0; i < 4; i++) {
        rpm_cmds[i] = computePIDandGetRPM(motors[i]);
      }
    }

    // Send commands
    sendMotorData(rpm_cmds[0], rpm_cmds[1], rpm_cmds[2], rpm_cmds[3]);

    lastSend = now;
  }

  if (millis() - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = millis();
    sendTelemetry();
  }

  if (millis() - lastDriveTelemetry >= DRIVE_TELEMETRY_INTERVAL) {
    lastDriveTelemetry = millis();
    sendDriveTelemetry();
  }

  delay(1);
}
