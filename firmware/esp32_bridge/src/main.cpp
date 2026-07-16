// =============================================================================
//  ESP32 Bridge — Pi 4  <->  Hoverboard mainboard (STM32F103, FOC firmware)
//
//  Role: the "brainstem". The Pi does all the thinking (ROS 2, Nav2, SLAM) but
//  NEVER talks to the motors directly. This layer enforces safety in hard
//  real time:
//    * Watchdog  — if the Pi goes silent (crash, SD stall, ROS node death)
//                  for WATCHDOG_MS, motors are commanded to zero. Non-optional.
//    * E-stop    — a hardware button (NC) pulls a GPIO; asserted = full stop,
//                  latched until explicitly cleared by the Pi.
//    * Feedback  — measured wheel speeds + battery + temp are relayed to the
//                  Pi for odometry and health monitoring.
//
//  Wiring (verify UART side against docs/bringup-checklist.md step 5!):
//    Serial2 (GPIO16=RX2, GPIO17=TX2) --- hoverboard USART (115200)
//    Serial  (USB, UART0)             --- Pi link (115200)   [binary protocol]
//    Serial1 (TX=GPIO4)               --- optional debug log; NEVER on the Pi link
//    GPIO25                           --- E-stop button, active-LOW, NC to GND
//
//  NOTE: one hoverboard mainboard drives BOTH wheels. 2WD = this one board.
//  cmd_vel is mapped to (speed=forward, steer=differential) by the Pi.
// =============================================================================

#include <Arduino.h>

// ---- Tunables ---------------------------------------------------------------
static const uint32_t HOVER_BAUD   = 115200;
static const uint32_t PI_BAUD      = 115200;
static const uint16_t WATCHDOG_MS  = 200;   // Pi silence -> stop
static const uint16_t TX_PERIOD_MS = 20;    // 50 Hz command rate to hoverboard
static const int16_t  SPEED_LIMIT  = 300;   // clamp; raise once you trust it
static const int16_t  STEER_LIMIT  = 300;
static const uint8_t  ESTOP_PIN    = 25;    // active-LOW
static const bool     DEBUG_ENABLE = false; // Serial1 debug (off = clean Pi link)

// ---- Hoverboard protocol ----------------------------------------------------
static const uint16_t HOVER_START = 0xABCD;

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;   // start ^ steer ^ speed
} HoverCommand;

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} HoverFeedback;

// ---- Pi link protocol (mirrors hoverboard style, own frames) ----------------
static const uint16_t PI_START = 0xABCD;

// Pi -> ESP32: each frame is also the heartbeat (resets the watchdog).
typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  speed;
  int16_t  steer;
  uint8_t  clear_estop;  // 1 = request clearing a latched e-stop
  uint8_t  _pad;
  uint16_t checksum;     // start ^ speed ^ steer ^ (clear_estop | _pad<<8)
} PiCommand;

// ESP32 -> Pi: telemetry + safety state.
typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  speedL_meas;
  int16_t  speedR_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint8_t  estop;        // 1 = e-stop asserted (latched)
  uint8_t  watchdog_ok;  // 1 = Pi heartbeat fresh
  uint16_t checksum;
} EspFeedback;

// ---- State ------------------------------------------------------------------
static uint32_t last_pi_ms      = 0;
static bool     estop_latched   = false;
static int16_t  cmd_speed       = 0;
static int16_t  cmd_steer       = 0;
static uint32_t last_tx_ms      = 0;
static HoverFeedback hover_fb   = {0};

// ---- Debug helper -----------------------------------------------------------
#define DBG(...) do { if (DEBUG_ENABLE) Serial1.printf(__VA_ARGS__); } while (0)

// ---- Hoverboard side --------------------------------------------------------
static void hoverSend(int16_t steer, int16_t speed) {
  HoverCommand c;
  c.start    = HOVER_START;
  c.steer    = steer;
  c.speed    = speed;
  c.checksum = (uint16_t)(c.start ^ c.steer ^ c.speed);
  Serial2.write((uint8_t *)&c, sizeof(c));
}

// Byte-wise state machine so we never block and never desync on a bad frame.
static void hoverReceive() {
  static uint8_t  buf[sizeof(HoverFeedback)];
  static uint16_t idx = 0;
  static uint8_t  prev = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (idx == 0) {
      // hunt for the start marker (little-endian 0xABCD => 0xCD then 0xAB)
      if (prev == 0xCD && b == 0xAB) {
        buf[0] = 0xCD; buf[1] = 0xAB; idx = 2;
      }
      prev = b;
      continue;
    }
    buf[idx++] = b;
    if (idx == sizeof(HoverFeedback)) {
      HoverFeedback fb;
      memcpy(&fb, buf, sizeof(fb));
      uint16_t chk = fb.start ^ fb.cmd1 ^ fb.cmd2 ^ fb.speedR_meas ^
                     fb.speedL_meas ^ fb.batVoltage ^ fb.boardTemp ^ fb.cmdLed;
      if (chk == fb.checksum) hover_fb = fb;   // accept only valid frames
      idx = 0; prev = 0;
    }
  }
}

// ---- Pi side ----------------------------------------------------------------
static void piReceive() {
  static uint8_t  buf[sizeof(PiCommand)];
  static uint16_t idx = 0;
  static uint8_t  prev = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (idx == 0) {
      if (prev == 0xCD && b == 0xAB) { buf[0] = 0xCD; buf[1] = 0xAB; idx = 2; }
      prev = b;
      continue;
    }
    buf[idx++] = b;
    if (idx == sizeof(PiCommand)) {
      PiCommand c;
      memcpy(&c, buf, sizeof(c));
      uint16_t payload2 = (uint16_t)(c.clear_estop | (c._pad << 8));
      uint16_t chk = c.start ^ c.speed ^ c.steer ^ payload2;
      if (chk == c.checksum) {
        last_pi_ms = millis();               // heartbeat refreshed
        cmd_speed  = constrain(c.speed, -SPEED_LIMIT, SPEED_LIMIT);
        cmd_steer  = constrain(c.steer, -STEER_LIMIT, STEER_LIMIT);
        if (c.clear_estop && digitalRead(ESTOP_PIN) == LOW) {
          estop_latched = false;             // clear only if button physically released (contact closed = LOW)
          DBG("estop cleared by Pi\n");
        }
      }
      idx = 0; prev = 0;
    }
  }
}

static void piSendFeedback() {
  EspFeedback f;
  f.start       = PI_START;
  f.speedL_meas = hover_fb.speedL_meas;
  f.speedR_meas = hover_fb.speedR_meas;
  f.batVoltage  = hover_fb.batVoltage;
  f.boardTemp   = hover_fb.boardTemp;
  f.estop       = estop_latched ? 1 : 0;
  f.watchdog_ok = (millis() - last_pi_ms <= WATCHDOG_MS) ? 1 : 0;
  f.checksum    = f.start ^ f.speedL_meas ^ f.speedR_meas ^ f.batVoltage ^
                  f.boardTemp ^ (uint16_t)(f.estop | (f.watchdog_ok << 8));
  Serial.write((uint8_t *)&f, sizeof(f));
}

// ---- Safety -----------------------------------------------------------------
// Returns true if it is safe to drive; also latches e-stop on button press.
static bool safetyOk() {
  // Fail-safe wiring: E-stop NC contact between pin and GND, INPUT_PULLUP.
  //   running (contact closed) -> pin LOW
  //   pressed OR wire cut/unplugged (contact open) -> pullup -> pin HIGH -> STOP
  // A broken wire therefore stops the robot, which is what we want.
  if (digitalRead(ESTOP_PIN) == HIGH) {
    if (!estop_latched) DBG("E-STOP asserted\n");
    estop_latched = true;
  }
  if (estop_latched) return false;
  if (millis() - last_pi_ms > WATCHDOG_MS) return false;  // Pi silent
  return true;
}

// ---- Arduino ----------------------------------------------------------------
void setup() {
  Serial.begin(PI_BAUD);                              // Pi link (USB)
  Serial2.begin(HOVER_BAUD, SERIAL_8N1, 16, 17);      // hoverboard
  if (DEBUG_ENABLE) Serial1.begin(115200, SERIAL_8N1, -1, 4);
  pinMode(ESTOP_PIN, INPUT_PULLUP);                   // NC button to GND

  // Start latched: nothing moves until the Pi has sent a fresh heartbeat.
  last_pi_ms    = millis() - WATCHDOG_MS - 1;
  estop_latched = false;
  DBG("bridge up\n");
}

void loop() {
  piReceive();
  hoverReceive();

  uint32_t now = millis();
  if (now - last_tx_ms >= TX_PERIOD_MS) {
    last_tx_ms = now;
    if (safetyOk()) {
      hoverSend(cmd_steer, cmd_speed);
    } else {
      hoverSend(0, 0);          // fail-safe: command zero, every cycle
      cmd_speed = cmd_steer = 0;
    }
    piSendFeedback();
  }
}
