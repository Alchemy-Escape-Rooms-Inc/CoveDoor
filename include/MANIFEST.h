/**
 * ============================================================================
 *  ALCHEMY ESCAPE ROOMS — FIRMWARE MANIFEST
 * ============================================================================
 *
 *  THIS FILE IS THE SINGLE SOURCE OF TRUTH.
 *
 *  It serves two masters simultaneously:
 *
 *    1. THE COMPILER — Every constant the firmware needs (pins, IPs, ports,
 *       thresholds, timing) is defined here as real C++ code. The firmware
 *       #includes this file and uses these values directly.
 *
 *    2. THE GRIMOIRE PARSER — A Python script running on M3 at 6 AM reads
 *       this file as plain text and extracts values tagged with @FIELD_NAME
 *       in the comments. Those values populate the WatchTower Grimoire
 *       device registry, wiring reference, and operations manual.
 *
 *  Because both systems read from the same lines, the documentation can
 *  never drift from the firmware. Change a pin number here, and the Grimoire
 *  updates automatically. There is no second file to keep in sync.
 *
 *  RULES:
 *    1. Every field marked [REQUIRED] must be filled in before deployment.
 *    2. Update this file FIRST when changing hardware, pins, or topics.
 *    3. The 6 AM parser looks for @TAG patterns — don't rename them.
 *    4. Descriptive-only sections (operations, quirks) are pure comments.
 *       Constants sections are real code + comment tags on the same line.
 *    5. This file is the sole source of configuration values — main.cpp
 *       should reference these constants, not hardcode its own.
 *
 *  LAST UPDATED: 2026-02-12
 *  MANIFEST VERSION: 2.0
 * ============================================================================
 */

#pragma once
#include <cstdint>

// ============================================================================
//  SECTION 1 — IDENTITY
// ============================================================================
//
// @MANIFEST:IDENTITY
// @PROP_NAME:        CoveDoor
// @INSTANCE_COUNT:   1
//
// @DESCRIPTION:      Secret automatic sliding door that transitions players
//                    from the Monkey Altar Room into the Cove. The door is
//                    disguised as part of the room wall — players do not know
//                    it exists until it opens. An XY160D H-bridge motor
//                    driver controls a DC motor that slides the door along a
//                    track. Two mechanical limit switches (with external
//                    pull-up resistors) define the open and closed positions.
//                    The door responds to OPEN, CLOSE, and STOP commands via
//                    MQTT, with smooth ramped motor acceleration and
//                    deceleration for theatrical effect.
//
// @ROOM:             Monkey Altar Room (transitions to Cove)
// @BOARD:            ESP32-DevKitC
// @FRAMEWORK:        Arduino (PlatformIO)
// @REPO:             https://github.com/Alchemy-Escape-Rooms-Inc/CoveSlidingDoor
// @BUILD_STATUS:     INSTALLED
// @CODE_HEALTH:      GOOD
// @WATCHTOWER:       COMPLIANT
// @END:IDENTITY

namespace manifest {

// ── Device Identity ─────────────────────────────────────────────────────────
inline constexpr const char* DEVICE_NAME      = "CoveDoor";       // @DEVICE_NAME  (MQTT client ID + topic base)
inline constexpr const char* FIRMWARE_VERSION  = "1.0.0";         // @FIRMWARE_VERSION


// ============================================================================
//  SECTION 2 — NETWORK CONFIGURATION
// ============================================================================
// @MANIFEST:NETWORK

// ── WiFi ────────────────────────────────────────────────────────────────────
inline constexpr const char* WIFI_SSID     = "AlchemyGuest";      // @WIFI_SSID
inline constexpr const char* WIFI_PASSWORD = "VoodooVacation5601"; // @WIFI_PASS

// ── MQTT Broker ─────────────────────────────────────────────────────────────
inline constexpr const char* MQTT_SERVER   = "10.1.10.115";       // @BROKER_IP
inline constexpr int         MQTT_PORT     = 1883;                // @BROKER_PORT
// MQTT Client ID: "CoveDoor" (matches DEVICE_NAME)

// ── Heartbeat ───────────────────────────────────────────────────────────────
inline constexpr unsigned long HEARTBEAT_INTERVAL = 30000;        // @HEARTBEAT_MS  (30 seconds)

//  ── TOPIC MAP ──────────────────────────────────────────────────────────────
//  Topics are built dynamically from DEVICE_NAME at runtime:
//    "MermaidsTale/" + DEVICE_NAME + "/{suffix}"
//
//  SUBSCRIPTIONS:
//  @SUBSCRIBE:  MermaidsTale/CoveDoor/command      | All commands (standard + door control)
//
//  PUBLICATIONS:
//  @PUBLISH:  MermaidsTale/CoveDoor/command         | PONG, STATUS, OK responses  | retain:no
//  @PUBLISH:  MermaidsTale/CoveDoor/status          | State changes + heartbeat   | retain:no
//  @PUBLISH:  MermaidsTale/CoveDoor/log             | Mirrored serial output      | retain:no
//  @PUBLISH:  MermaidsTale/CoveDoor/limit           | Limit switch events         | retain:no
//
//  NOTE: PONG and STATUS responses publish on /command (not /status).
//        This differs from JungleDoor which publishes PONG on /status.
//
//  SUPPORTED COMMANDS (via /command topic):
//  @COMMAND:  PING          | Responds PONG on /command topic         | Health check
//  @COMMAND:  STATUS        | Sends state report on /command topic    | Full diagnostic
//  @COMMAND:  RESET         | Stops motor, reboots ESP32              | Also accepts REBOOT, RESTART
//  @COMMAND:  PUZZLE_RESET  | Stops motor, re-reads limits, resets state | No reboot
//  @COMMAND:  OPEN          | Opens the door (ramp up → full → ramp down)
//  @COMMAND:  CLOSE         | Closes the door (ramp up → full → ramp down)
//  @COMMAND:  STOP          | Emergency stop — kills motor immediately
//
//  LIMIT SWITCH EVENTS (published on /limit topic):
//  @LIMIT_EVENT:  LIMIT_OPEN_HIT      | Door reached fully open position
//  @LIMIT_EVENT:  LIMIT_CLOSED_HIT    | Door reached fully closed position
//  @LIMIT_EVENT:  LIMIT_OPEN_CLEAR    | Door moved away from open limit
//  @LIMIT_EVENT:  LIMIT_CLOSED_CLEAR  | Door moved away from closed limit
//
//  STATUS MESSAGES (published on /status topic):
//  @STATUS_MSG:  ONLINE          | Sent on boot and MQTT reconnect
//  @STATUS_MSG:  OPENING         | Door motor started, opening direction
//  @STATUS_MSG:  CLOSING         | Door motor started, closing direction
//  @STATUS_MSG:  OPEN            | Door reached open position (limit or timeout)
//  @STATUS_MSG:  CLOSED          | Door reached closed position (limit or timeout)
//  @STATUS_MSG:  STOPPED         | Motor stopped via STOP command
//  @STATUS_MSG:  ALREADY_OPEN    | OPEN command received but door already open
//  @STATUS_MSG:  ALREADY_CLOSED  | CLOSE command received but door already closed
//  @STATUS_MSG:  HEARTBEAT:...   | Periodic heartbeat with state, uptime, RSSI
//
// @END:NETWORK


// ============================================================================
//  SECTION 3 — PIN CONFIGURATION
// ============================================================================
// @MANIFEST:PINS

// ── Motor Driver (XY160D H-Bridge) ───────────────────────────────────────────
inline constexpr int MOTOR_IN1 = 2;                               // @PIN:IN1    | XY160D IN1 — direction control A
inline constexpr int MOTOR_IN2 = 5;                               // @PIN:IN2    | XY160D IN2 — direction control B
inline constexpr int MOTOR_ENA = 4;                               // @PIN:ENA    | XY160D ENA — PWM speed control

// ── Limit Switches ──────────────────────────────────────────────────────────
inline constexpr int LIMIT_OPEN   = 16;                           // @PIN:LIMIT_OPEN   | Magnetic reed switch, INPUT_PULLUP, active LOW
inline constexpr int LIMIT_CLOSED = 32;                           // @PIN:LIMIT_CLOSED | Magnetic reed switch, INPUT_PULLUP, active LOW

// @END:PINS


// ============================================================================
//  SECTION 4 — MOTOR CONFIGURATION
// ============================================================================
// @MANIFEST:MOTOR

inline constexpr int MOTOR_SPEED = 150;                           // @MOTOR:SPEED | PWM duty 0-255 (150 ≈ 59%)

// ── PWM Configuration ───────────────────────────────────────────────────────
inline constexpr int PWM_CHANNEL   = 0;                           // @PWM:CHANNEL    | LEDC channel for ENA speed control
inline constexpr int PWM_FREQ      = 5000;                        // @PWM:FREQ       | 5kHz PWM frequency
inline constexpr int PWM_RESOLUTION = 8;                          // @PWM:RESOLUTION | 8-bit (0-255)

// ── Door Movement Timing ────────────────────────────────────────────────────
inline constexpr int DOOR_RAMP_UP_MS    = 500;                    // @DOOR:RAMP_UP    | 0.5s acceleration to full speed
inline constexpr int DOOR_TIMEOUT_MS    = 8000;                   // @DOOR:TIMEOUT    | 8s safety timeout if limit switch not hit

// @END:MOTOR


// ============================================================================
//  SECTION 5 — SENSOR THRESHOLDS
// ============================================================================
// @MANIFEST:THRESHOLDS

// ── Debounce ────────────────────────────────────────────────────────────────
inline constexpr int LIMIT_DEBOUNCE_MS = 150;                     // @DEBOUNCE:LIMIT  | 150ms debounce for both limit switches

// @END:THRESHOLDS


// ============================================================================
//  SECTION 6 — TIMING CONSTANTS
// ============================================================================
// @MANIFEST:TIMING

inline constexpr unsigned long WIFI_CHECK_INTERVAL     = 30000;   // @TIMING:WIFI_CHECK     | Check WiFi connection every 30s
inline constexpr unsigned long MQTT_RECONNECT_INTERVAL = 5000;    // @TIMING:MQTT_RECONNECT | Retry MQTT connection every 5s

// @END:TIMING

} // namespace manifest


// ============================================================================
//  SECTION 7 — COMPONENTS
// ============================================================================
//
// @MANIFEST:COMPONENTS
//
// @COMPONENT:  XY160D H-Bridge Motor Driver
//   @PURPOSE:  Controls the DC motor that slides the door along its track
//   @DETAIL:   DIR + PWM interface (IN1/IN2 for direction, ENA for speed).
//              IN1=HIGH + IN2=LOW drives the open direction, IN1=LOW +
//              IN2=HIGH drives the close direction. ENA receives PWM for
//              speed control. Motor runs at 59% duty (150/255) with smooth
//              ramp-up for theatrical door movement. Opto-isolated inputs
//              protect the ESP32 from motor noise.
//
// @COMPONENT:  DC Sliding Door Motor
//   @PURPOSE:  Physically moves the door panel along a track
//   @DETAIL:   Driven by XY160D. Opens via IN1, closes via IN2.
//              Total travel time approximately 4 seconds with ramping.
//
// @COMPONENT:  Magnetic Reed Switch (Open Position)
//   @PURPOSE:  Detects when door has fully opened
//   @DETAIL:   Pin 16, INPUT_PULLUP, active LOW. Magnet mounted on door panel.
//
// @COMPONENT:  Magnetic Reed Switch (Closed Position)
//   @PURPOSE:  Detects when door has fully closed
//   @DETAIL:   Pin 32, INPUT_PULLUP, active LOW. Magnet mounted on door panel.
//
// @END:COMPONENTS


// ============================================================================
//  SECTION 8 — OPERATIONS
// ============================================================================
//
// @MANIFEST:OPERATIONS
//
//  ── PHYSICAL LOCATION ──────────────────────────────────────────────────────
//
// @LOCATION:  Hidden above the door frame. The ESP32 and XY160D motor driver
//             are mounted in the space directly above the CoveDoor. Access
//             from above.
//
//  ── RESET PROCEDURES ───────────────────────────────────────────────────────
//
// @RESET:SOFTWARE
//   Send "RESET" (or "REBOOT" or "RESTART") to MermaidsTale/CoveDoor/command
//   Device responds "OK" on /command, stops motor, then reboots
//   After reboot: reconnects WiFi, reconnects MQTT, reads limit switches
//   to determine initial door position, publishes ONLINE
//   Expected recovery time: 10-15 seconds
//
// @RESET:PUZZLE
//   Send "PUZZLE_RESET" to MermaidsTale/CoveDoor/command
//   Stops motor, re-reads limit switches, sets state to match physical
//   position (CLOSED if closed limit active, OPEN if open limit active,
//   STOPPED if neither). No reboot, no WiFi/MQTT reconnect.
//   Responds "OK" on /command topic.
//   Use between game sessions to sync state with physical door position.
//
// @RESET:HARDWARE
//   The ESP32 and BTS7960 are hidden above the door frame. Access from above,
//   disconnect and reconnect power to the ESP32. After power-on, monitor
//   MermaidsTale/CoveDoor/status for ONLINE message. The door will read its
//   limit switches on boot to determine its current position.
//
//  ── DOOR OPERATION ─────────────────────────────────────────────────────────
//
// @OPERATION:OPEN
//   Send "OPEN" to MermaidsTale/CoveDoor/command
//   Sets IN1=HIGH, IN2=LOW, ramps ENA PWM up over 0.5s to duty 150.
//   Publishes "OPENING" immediately, then "OPEN" when complete.
//   Stops early if open limit switch triggers.
//   Falls back to 8-second timer if limit switch doesn't fire.
//
// @OPERATION:CLOSE
//   Send "CLOSE" to MermaidsTale/CoveDoor/command
//   Sets IN1=LOW, IN2=HIGH, ramps ENA PWM up over 0.5s to duty 150.
//   Publishes "CLOSING" immediately, then "CLOSED" when complete.
//   Stops early if closed limit switch triggers.
//   Falls back to 8-second timer if limit switch doesn't fire.
//
// @OPERATION:EMERGENCY_STOP
//   Send "STOP" to MermaidsTale/CoveDoor/command
//   ENA immediately set to 0, IN1 and IN2 set LOW — no ramp-down.
//   Publishes "STOPPED" on /status.
//   Door remains in whatever position it stopped at.
//
//  ── TEST PROCEDURE ─────────────────────────────────────────────────────────
//
// @TEST:STEP1  Send PING to /command → expect PONG on /command (confirms MQTT)
// @TEST:STEP2  Send STATUS to /command → expect state, uptime, RSSI, version, limit states on /command
// @TEST:STEP3  Send OPEN to /command → door should slide open, expect OPENING then OPEN on /status
// @TEST:STEP4  Verify LIMIT_OPEN_HIT appears on /limit topic when door reaches open position
// @TEST:STEP5  Send CLOSE to /command → door should slide closed, expect CLOSING then CLOSED on /status
// @TEST:STEP6  Verify LIMIT_CLOSED_HIT appears on /limit topic when door reaches closed position
// @TEST:STEP7  Send STOP during movement → motor should stop immediately
// @TEST:STEP8  Send PUZZLE_RESET → expect OK on /command, state syncs to physical position
//
//  ── KNOWN QUIRKS ───────────────────────────────────────────────────────────
//
// @QUIRK:ESP32_PIN_CHANGE
//   The limit switches were originally on GPIO 38 and 39 (from the ESP32-S3
//   version). When the board was changed to a regular ESP32, the pins were
//   first moved to GPIO 32 and 33, but GPIO 33's internal pull-up was
//   defective on the installed board (stuck at 0.55V with nothing connected).
//   Limit switches were also changed from mechanical to magnetic reed switches.
//   Final pin assignment: LIMIT_OPEN on GPIO 16, LIMIT_CLOSED on GPIO 32.
//
// @QUIRK:NO_WATCHDOG
//   This firmware does not implement a hardware watchdog timer. If the main
//   loop hangs (e.g., WiFi blocking, MQTT stall), the device will not
//   auto-recover. A manual RESET command or hardware power cycle is required.
//   Consider adding esp_task_wdt for future versions.
//
// @QUIRK:TIMEOUT_BACKUP
//   The 4-second movement timer serves as a backup timeout. The limit
//   switches are the primary stop mechanism and are working reliably. If a
//   limit switch fails to trigger within 4 seconds, the motor stops anyway
//   and the firmware logs "[TIMEOUT] Door open/close timeout - no limit hit"
//   on the /log topic. If you see timeout messages, check the limit switches.
//
// @QUIRK:PONG_ON_COMMAND
//   PONG and STATUS responses are published on the /command topic, not
//   /status. This is different from JungleDoor (which publishes PONG on
//   /status). WatchTower System Checker should listen on /command for
//   health check responses from this device.
//
// @QUIRK:LEDC_API_VERSION
//   The code uses the older ledcSetup()/ledcAttachPin() API for the ENA
//   PWM pin, which is correct for the regular ESP32. The newer
//   ledcAttach() API (used by JungleDoor) is an ESP32-S3/Arduino 3.x
//   feature. Do not "modernize" this code to the newer API without
//   verifying board compatibility.
//
//  ── TROUBLESHOOTING LOG (2026-02-14) ───────────────────────────────────────
//
// @QUIRK:GPIO5_BOOT_CRASH
//   SYMPTOM: Firmware runs normally when powered via USB alone. The moment
//   the ESP32 is seated into the screw terminal breakout (even with NO
//   external power on the terminal), the board boot-loops with:
//     rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
//     invalid header: 0xffffffff
//   CAUSE: GPIO 5 is a strapping pin on the regular ESP32. It controls
//   SDIO timing during the ROM bootloader's first microseconds — before
//   any user code runs. When the screw terminal makes contact with GPIO 5
//   (IN2 pin), it pulls the pin to an unexpected state during boot,
//   causing the ESP32 to attempt flash reads at the wrong voltage (1.8V
//   instead of 3.3V). This produces the 0xffffffff invalid headers.
//   FIX: Move IN2 off GPIO 5 to a non-strapping pin. Strapping pins on
//   the regular ESP32 to AVOID for signals with external connections:
//   GPIO 0, 2, 5, 12, 15. This does NOT affect the ESP32-S3 which has
//   different strapping pins.
//   NOTE: The code compiles and runs fine — this is purely a hardware
//   boot-level issue. pinMode() and digitalWrite() work correctly on
//   GPIO 5 after boot. The problem is only during the first microseconds
//   of power-on when the ROM bootloader reads strapping pin states.
//
// @QUIRK:GPIO33_DEFECTIVE_PULLUP
//   SYMPTOM: GPIO 33's internal pull-up only reaches 0.55V instead of
//   3.3V, even with absolutely nothing connected to the pin. Measured
//   with multimeter — confirmed defective on this specific board.
//   CAUSE: Defective internal pull-up resistor on this particular ESP32
//   unit. Not a code issue, not a wiring issue — the silicon is bad on
//   this one pin. INPUT_PULLUP is configured correctly in firmware but
//   the hardware cannot deliver the expected voltage.
//   EFFECT: Any sensor on GPIO 33 (limit switch, reed switch, etc.)
//   will never read a clean HIGH when the switch is open. The pin sits
//   in the indeterminate zone and triggers false readings.
//   FIX: Moved LIMIT_CLOSED from GPIO 33 to GPIO 32. If GPIO 33 must
//   be used on this board, add an external 10K pull-up resistor from
//   the pin to 3.3V to overpower the defective internal pull-up.
//   NOTE: This is specific to this individual ESP32 board. Replacement
//   boards will likely have a working GPIO 33. Document the board if
//   it gets swapped out.
//
// @END:OPERATIONS


// ============================================================================
//  SECTION 9 — DEPENDENCIES
// ============================================================================
//
// @MANIFEST:DEPENDENCIES
//
// @LIB:  WiFi              | ESP32 WiFi driver          | Built-in
// @LIB:  PubSubClient      | MQTT client                | v2.8+
//
// @END:DEPENDENCIES


// ============================================================================
//  SECTION 10 — WIRING SUMMARY
// ============================================================================
//
// @MANIFEST:WIRING
//
//   ESP32 Pin 2  (IN1) ───────── XY160D IN1 (direction control A)
//   ESP32 Pin 5  (IN2) ───────── XY160D IN2 (direction control B)
//   ESP32 Pin 4  (ENA) ───────── XY160D ENA (PWM speed control)
//
//   ESP32 Pin 16 ──────────────── Magnetic Reed Switch (OPEN position)
//                                 Closes to GND when magnet is nearby
//                                 INPUT_PULLUP, active LOW
//
//   ESP32 Pin 32 ──────────────── Magnetic Reed Switch (CLOSED position)
//                                 Closes to GND when magnet is nearby
//                                 INPUT_PULLUP, active LOW
//
//   XY160D VCC  ───────────────── Motor power supply (12V/24V)
//   XY160D GND  ───────────────── Common ground with ESP32
//   XY160D MOTOR OUT ──────────── DC sliding door motor
//
//   ESP32 Power ───────────────── USB, hidden above the door frame
//
//   Physical Location: Hidden above the door frame, accessible from above
//
// @END:WIRING
