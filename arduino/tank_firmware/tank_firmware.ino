/**
 * tank_firmware.ino
 *
 * Arduino Uno firmware for the Makis mecanum tank robot.
 *
 * Hardware
 * --------
 * - Adafruit Motor Shield V2 (PCA9685 + 2×TB6612)
 * - 4× 37 mm DC motors with hall-effect encoders (1:90 gear)
 * - PS2 wireless controller receiver (SPI pins D10-D13)
 *
 * Pin mapping
 * -----------
 * D2  → ENC_FL_A  (INT0 — quadrature phase A, Front-Left)
 * D3  → ENC_FR_A  (INT1 — quadrature phase A, Front-Right)
 * D4  → ENC_RL_A  (Timer2 ISR poll — Rear-Left)
 * D5  → ENC_RR_A  (Timer2 ISR poll — Rear-Right)
 * D6  → ENC_FL_B
 * D7  → ENC_FR_B
 * D8  → ENC_RL_B
 * D9  → ENC_RR_B
 * D10 → PS2_SEL (SS)
 * D11 → PS2_CMD (MOSI)
 * D12 → PS2_DAT (MISO)
 * D13 → PS2_CLK (SCK)
 * A4  → SDA (Motor Shield I²C)
 * A5  → SCL (Motor Shield I²C)
 *
 * Serial protocol (115200 baud)
 * ------------------------------
 * RPi → Arduino:  CMD:<FL>:<FR>:<RL>:<RR>\n   (speed -255..255)
 * Arduino → RPi:  ENC:<FL>:<FR>:<RL>:<RR>:<dt_ms>\n  (ticks + Δt)
 *                 PS2:<lx>:<ly>:<rx>:<ry>:<btns>\n    (joystick)
 *                 INFO:<message>\n
 *
 * Required libraries (install via Library Manager)
 * -------------------------------------------------
 * - Adafruit Motor Shield V2  (Adafruit_MotorShield)
 * - PS2X_lib by Bill Porter
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PS2X_lib.h>
#include <util/atomic.h>

// ─── Configuration ────────────────────────────────────────────
#define SERIAL_BAUD       115200
#define CMD_TIMEOUT_MS     500   // ms before motors are stopped (watchdog)
#define ENC_REPORT_MS       50   // encoder publish period  (20 Hz)
#define PS2_POLL_MS         20   // PS2 poll period         (50 Hz)
#define MAX_SPEED          200   // hard limit out of 255

// ─── Encoder pins ─────────────────────────────────────────────
#define ENC_FL_A   2
#define ENC_FR_A   3
#define ENC_RL_A   4
#define ENC_RR_A   5
#define ENC_FL_B   6
#define ENC_FR_B   7
#define ENC_RL_B   8
#define ENC_RR_B   9

// ─── PS2 pins ─────────────────────────────────────────────────
#define PS2_SEL   10
#define PS2_CMD   11
#define PS2_DAT   12
#define PS2_CLK   13

// ─── PS2 stick dead-zone ──────────────────────────────────────
#define JOY_DEADZONE  10   // ignore values within ±10 of centre (128)

// ─── Globals ──────────────────────────────────────────────────
Adafruit_MotorShield shield;
Adafruit_DCMotor *motorFL;
Adafruit_DCMotor *motorFR;
Adafruit_DCMotor *motorRL;
Adafruit_DCMotor *motorRR;

PS2X ps2x;
bool ps2Ready = false;

// Encoder counts (updated in ISR / timer poll)
volatile long encFL = 0;
volatile long encFR = 0;
volatile long encRL = 0;
volatile long encRR = 0;

// Previous state for Timer2 poll (RL, RR)
volatile uint8_t prevRL_A = 0;
volatile uint8_t prevRR_A = 0;

// Previous encoder counts snapshot for delta calculation
long snapFL = 0, snapFR = 0, snapRL = 0, snapRR = 0;

// Timing
unsigned long lastCmdTime   = 0;
unsigned long lastEncReport = 0;
unsigned long lastPS2Poll   = 0;

// Mode: false = PS2 manual, true = ROS autonomous
bool autoMode = false;

// ─── Motor helpers ────────────────────────────────────────────
static void driveMotor(Adafruit_DCMotor *m, int spd) {
    spd = constrain(spd, -255, 255);
    if (spd > 0) {
        m->setSpeed((uint8_t)spd);
        m->run(FORWARD);
    } else if (spd < 0) {
        m->setSpeed((uint8_t)(-spd));
        m->run(BACKWARD);
    } else {
        m->setSpeed(0);
        m->run(RELEASE);
    }
}

static void stopAll() {
    motorFL->run(BRAKE);
    motorFR->run(BRAKE);
    motorRL->run(BRAKE);
    motorRR->run(BRAKE);
}

static void setMecanum(int fl, int fr, int rl, int rr) {
    driveMotor(motorFL, constrain(fl, -MAX_SPEED, MAX_SPEED));
    driveMotor(motorFR, constrain(fr, -MAX_SPEED, MAX_SPEED));
    driveMotor(motorRL, constrain(rl, -MAX_SPEED, MAX_SPEED));
    driveMotor(motorRR, constrain(rr, -MAX_SPEED, MAX_SPEED));
}

// ─── Encoder ISRs ─────────────────────────────────────────────
// FL — INT0 (D2)
ISR(INT0_vect) {
    if (digitalRead(ENC_FL_B)) encFL++;
    else                        encFL--;
}

// FR — INT1 (D3)
ISR(INT1_vect) {
    if (digitalRead(ENC_FR_B)) encFR++;
    else                        encFR--;
}

// Timer2 CompA @ ~500 Hz — polls RL and RR encoder A-pins
ISR(TIMER2_COMPA_vect) {
    uint8_t rl_a = (PIND >> 4) & 1;  // D4
    uint8_t rr_a = (PIND >> 5) & 1;  // D5

    if (rl_a != prevRL_A) {
        if ((PINB >> 0) & 1) encRL++;  // ENC_RL_B = D8 → PORTB bit0
        else                 encRL--;
        prevRL_A = rl_a;
    }
    if (rr_a != prevRR_A) {
        if ((PINB >> 1) & 1) encRR++;  // ENC_RR_B = D9 → PORTB bit1
        else                 encRR--;
        prevRR_A = rr_a;
    }
}

// ─── Timer2 setup (500 Hz with /128 prescaler) ────────────────
static void setupTimer2() {
    TCCR2A = (1 << WGM21);          // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS20); // prescaler 128
    OCR2A  = 249;                   // 16 MHz / 128 / 250 = 500 Hz
    TIMSK2 = (1 << OCIE2A);
}

// ─── Serial command parser ────────────────────────────────────
static void parseSerial() {
    static char buf[48];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            buf[idx] = '\0';
            idx = 0;

            if (strncmp(buf, "CMD:", 4) == 0) {
                int fl, fr, rl, rr;
                if (sscanf(buf + 4, "%d:%d:%d:%d", &fl, &fr, &rl, &rr) == 4) {
                    autoMode    = true;
                    lastCmdTime = millis();
                    setMecanum(fl, fr, rl, rr);
                }
            } else if (strcmp(buf, "STOP") == 0) {
                stopAll();
                autoMode = false;
            } else if (strcmp(buf, "AUTO") == 0) {
                autoMode = true;
            } else if (strcmp(buf, "MANUAL") == 0) {
                autoMode = false;
            }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// ─── Encoder report ───────────────────────────────────────────
static void reportEncoders() {
    long fl, fr, rl, rr;
    unsigned long now = millis();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        fl = encFL; fr = encFR; rl = encRL; rr = encRR;
    }
    long dt = (long)(now - lastEncReport);
    lastEncReport = now;

    // Send delta ticks since last report
    Serial.print("ENC:");
    Serial.print(fl - snapFL); Serial.print(':');
    Serial.print(fr - snapFR); Serial.print(':');
    Serial.print(rl - snapRL); Serial.print(':');
    Serial.print(rr - snapRR); Serial.print(':');
    Serial.println(dt);

    snapFL = fl; snapFR = fr; snapRL = rl; snapRR = rr;
}

// ─── PS2 poll ─────────────────────────────────────────────────
static void pollPS2() {
    if (!ps2Ready) return;
    ps2x.read_gamepad(false, 0);

    // Buttons bitmask (active-HIGH convenience flags)
    uint16_t btns = ps2x.ButtonDataByte();

    byte lx = ps2x.Analog(PSS_LX);
    byte ly = ps2x.Analog(PSS_LY);
    byte rx = ps2x.Analog(PSS_RX);
    byte ry = ps2x.Analog(PSS_RY);

    // START = toggle autonomous mode
    if (ps2x.ButtonPressed(PSB_START)) {
        autoMode = !autoMode;
        Serial.print("INFO:mode=");
        Serial.println(autoMode ? "AUTO" : "MANUAL");
    }

    // SELECT = emergency stop
    if (ps2x.ButtonPressed(PSB_SELECT)) {
        stopAll();
        Serial.println("INFO:ESTOP");
    }

    // Publish raw PS2 data for ROS to consume
    Serial.print("PS2:");
    Serial.print(lx); Serial.print(':');
    Serial.print(ly); Serial.print(':');
    Serial.print(rx); Serial.print(':');
    Serial.print(ry); Serial.print(':');
    Serial.println(btns);

    // Manual driving when not in auto mode
    if (!autoMode) {
        // Dead-zone
        int dlx = (int)lx - 128;  if (abs(dlx) < JOY_DEADZONE) dlx = 0;
        int dly = (int)ly - 128;  if (abs(dly) < JOY_DEADZONE) dly = 0;
        int drx = (int)rx - 128;  if (abs(drx) < JOY_DEADZONE) drx = 0;

        // Mecanum IK:  LY=fwd/back  LX=strafe  RX=rotate
        // Invert LY because PS2 Y-axis is inverted
        int vx =  -dly;   // fwd/back
        int vy =   dlx;   // strafe
        int omega = drx;  // rotation

        // Speed scale 128 → MAX_SPEED
        int fl = (int)((vx - vy - omega) * MAX_SPEED / 128);
        int fr = (int)((vx + vy + omega) * MAX_SPEED / 128);
        int rl = (int)((vx + vy - omega) * MAX_SPEED / 128);
        int rr = (int)((vx - vy + omega) * MAX_SPEED / 128);

        setMecanum(fl, fr, rl, rr);
    }
}

// ─── setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);

    // Encoder input pins
    uint8_t encPins[] = {ENC_FL_A, ENC_FR_A, ENC_RL_A, ENC_RR_A,
                         ENC_FL_B, ENC_FR_B, ENC_RL_B, ENC_RR_B};
    for (uint8_t p : encPins) {
        pinMode(p, INPUT_PULLUP);
    }

    // Hardware external interrupts for FL and FR
    attachInterrupt(digitalPinToInterrupt(ENC_FL_A), nullptr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_A), nullptr, CHANGE);
    // Use raw EICRA/EIMSK for CHANGE on both INT0 and INT1
    EICRA = (1 << ISC10) | (1 << ISC00);  // CHANGE on INT0 and INT1
    EIMSK = (1 << INT1)  | (1 << INT0);

    // Timer2 for RL / RR polling at 500 Hz
    setupTimer2();

    // Motor shield
    shield = Adafruit_MotorShield();
    if (!shield.begin()) {
        Serial.println("INFO:shield_fail");
        while (1);
    }
    motorFL = shield.getMotor(1);
    motorFR = shield.getMotor(2);
    motorRL = shield.getMotor(3);
    motorRR = shield.getMotor(4);
    stopAll();

    // PS2 controller
    int err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT,
                                  false, false);
    if (err == 0) {
        ps2Ready = true;
        Serial.println("INFO:ps2_ok");
    } else {
        Serial.print("INFO:ps2_err=");
        Serial.println(err);
    }

    sei();  // global interrupt enable
    Serial.println("INFO:boot_ok");
    lastCmdTime = millis();
}

// ─── loop ─────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // Parse incoming serial commands
    parseSerial();

    // Watchdog: stop motors if no CMD received while in auto mode
    if (autoMode && (now - lastCmdTime > CMD_TIMEOUT_MS)) {
        stopAll();
        Serial.println("INFO:watchdog_stop");
        autoMode = false;
    }

    // Encoder report
    if (now - lastEncReport >= ENC_REPORT_MS) {
        reportEncoders();
    }

    // PS2 poll
    if (now - lastPS2Poll >= PS2_POLL_MS) {
        lastPS2Poll = now;
        pollPS2();
    }
}
