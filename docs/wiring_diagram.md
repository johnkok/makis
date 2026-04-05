# Wiring Diagram

## System Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        POWER RAIL                                            │
│                                                                              │
│  LiPo 3S (11.1 V)──┬──[5A BEC 5V]──────────────────── RPi 5 (USB-C 5V/5A) │
│                     │                                                         │
│                     ├──[Arduino DC Jack 7-12V]──────── Arduino Uno          │
│                     │                                                         │
│                     └──[Motor Shield VIN]─────────────── TB6612 Vmot        │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Raspberry Pi 5 Connections

```
Raspberry Pi 5
│
├── USB-A port 0  ──── Arduino Uno (USB-B cable)  /dev/ttyACM0
├── USB-A port 1  ──── HLS-LFCD2 LiDAR           /dev/ttyUSB0
├── CSI connector ──── Pi Camera v2 (ribbon)
│
├── GPIO 2 (SDA)  ─┬─  LSM303DLHC SDA
│                  └─  L3GD20 SDA
│
├── GPIO 3 (SCL)  ─┬─  LSM303DLHC SCL
│                  └─  L3GD20 SCL
│
├── GPIO 17       ──── 3.3V ─┬─ LSM303DLHC VCC
│   (or 3V3 pin)             └─ L3GD20 VCC
└── GND           ─────────── IMU GNDs
```

### I²C Device Addresses on RPi 5

| Device | I²C Address | Notes |
|--------|------------|-------|
| LSM303DLHC Accel | 0x19 | SA0 pulled HIGH |
| LSM303DLHC Mag | 0x1E | Fixed |
| L3GD20 Gyro | 0x6B | SDO pulled HIGH |

---

## Arduino Uno + Motor Servo Shield

```
                    ┌──────────────────────────────────┐
                    │     Motor Servo Shield           │
                    │  (PCA9685 + 2×TB6612)            │
                    │                                  │
  Arduino Uno       │  I²C (A4=SDA, A5=SCL) → PCA9685 │
  ┌─────────┐       │  I²C addr: 0x40                 │
  │ A4 SDA──┼───────┼────► PCA9685 SDA                │
  │ A5 SCL──┼───────┼────► PCA9685 SCL                │
  │         │       │                                  │
  │ D2  ────┼───────┼────► ENC_FL_A  (INT0)           │
  │ D3  ────┼───────┼────► ENC_FR_A  (INT1)           │
  │ D4  ────┼───────┼────► ENC_RL_A  (PCINT)          │
  │ D5  ────┼───────┼────► ENC_RR_A  (PCINT)          │
  │ D6  ────┼───────┼────► ENC_FL_B                   │
  │ D7  ────┼───────┼────► ENC_FR_B                   │
  │ D8  ────┼───────┼────► ENC_RL_B                   │
  │ D9  ────┼───────┼────► ENC_RR_B                   │
  │         │       │                                  │
  │ D10 ────┼───────┼────► PS2_SEL  (SS)              │
  │ D11 ────┼───────┼────► PS2_CMD  (MOSI)            │
  │ D12 ────┼───────┼────► PS2_DAT  (MISO)            │
  │ D13 ────┼───────┼────► PS2_CLK  (SCK)             │
  └─────────┘       └──────────────────────────────────┘
```

### PCA9685 PWM Channel Assignment

| PCA9685 Ch | Signal | TB6612 Pin | Motor |
|-----------|--------|-----------|-------|
| CH 0 | PWM | PWMA — TB6612 #1 | Motor 1 (FL) speed |
| CH 1 | IN1 | AIN1 — TB6612 #1 | Motor 1 (FL) dir A |
| CH 2 | IN2 | AIN2 — TB6612 #1 | Motor 1 (FL) dir B |
| CH 3 | PWM | PWMB — TB6612 #1 | Motor 2 (FR) speed |
| CH 4 | IN1 | BIN1 — TB6612 #1 | Motor 2 (FR) dir A |
| CH 5 | IN2 | BIN2 — TB6612 #1 | Motor 2 (FR) dir B |
| CH 6 | — | STBY — TB6612 #1 | Enable chip 1 |
| CH 8 | PWM | PWMA — TB6612 #2 | Motor 3 (RL) speed |
| CH 9 | IN1 | AIN1 — TB6612 #2 | Motor 3 (RL) dir A |
| CH 10 | IN2 | AIN2 — TB6612 #2 | Motor 3 (RL) dir B |
| CH 11 | PWM | PWMB — TB6612 #2 | Motor 4 (RR) speed |
| CH 12 | IN1 | BIN1 — TB6612 #2 | Motor 4 (RR) dir A |
| CH 13 | IN2 | BIN2 — TB6612 #2 | Motor 4 (RR) dir B |
| CH 14 | — | STBY — TB6612 #2 | Enable chip 2 |

> **Note:** The Adafruit Motor Shield V2 library (used in firmware) handles all of the
> above channel assignments automatically. If using a clone shield with identical layout
> the Adafruit library still works. Regenerate the channel table if using a different
> shield layout.

---

## Motor Encoder Wiring (37 mm, 1:90, Hall encoder)

Each motor has 6 wires (standard colour coding):

| Motor Wire | Colour | Arduino Pin |
|-----------|--------|-------------|
| Motor + | Red | M_OUT_A+ (shield terminal) |
| Motor − | Black | M_OUT_A− (shield terminal) |
| Encoder VCC | Blue / White | 5 V |
| Encoder GND | Green | GND |
| Encoder A (Phase A) | Yellow | See table below |
| Encoder B (Phase B) | White / Purple | See table below |

| Motor | Position | ENC_A | ENC_B |
|-------|----------|-------|-------|
| M1 | Front-Left (FL) | D2 | D6 |
| M2 | Front-Right (FR) | D3 | D7 |
| M3 | Rear-Left (RL) | D4 | D8 |
| M4 | Rear-Right (RR) | D5 | D9 |

---

## HLS-LFCD2 LiDAR

```
HLS-LFCD2
├── USB  ──────────────── RPi 5 USB-A  (/dev/ttyUSB0, 230400 baud)
├── 5V   ──────────────── RPi 5 5V pin  (or dedicated USB power)
└── GND  ──────────────── RPi 5 GND
```

> Mount the LiDAR flat on top of the chassis, centred left-to-right.  
> The USB cable exit points toward the rear.

---

## PS2 Wireless Receiver (on Motor Shield header)

```
PS2 Receiver Dongle
├── DAT  ──── D12 (MISO)
├── CMD  ──── D11 (MOSI)
├── VCC  ──── 3.3 V
├── GND  ──── GND
├── SEL  ──── D10 (SS/CS)
└── CLK  ──── D13 (SCK)
```

---

## Pi Camera v2

```
Pi Camera v2
└── FFC ribbon ──── RPi 5 CSI-2 connector (port CAM1)
```

Enable in `/boot/firmware/config.txt`:
```
camera_auto_detect=1
```

---

## IMU Breakout Boards (I²C)

```
LSM303DLHC                     L3GD20
├── VCC ─── RPi 3.3 V          ├── VCC ─── RPi 3.3 V
├── GND ─── RPi GND            ├── GND ─── RPi GND
├── SDA ─── RPi GPIO 2 (SDA)   ├── SDA ─── RPi GPIO 2
├── SCL ─── RPi GPIO 3 (SCL)   ├── SCL ─── RPi GPIO 3
├── INT ─── (not connected)    ├── INT1 ── (not connected)
└── SA0 ─── 3.3 V (addr 0x19) └── SDO ─── 3.3 V (addr 0x6B)
```

Connect both boards to the **same I²C bus**.  
Add **4.7 kΩ pull-up resistors** on SDA and SCL lines if not already on the breakout.

---

## Power Budget

| Component | Voltage | Current (max) |
|-----------|---------|--------------|
| RPi 5 | 5 V | 3 A |
| Arduino Uno | 7–12 V | 0.1 A |
| 4× 37mm motors | 6–12 V | 4× 0.7 A = 2.8 A |
| Motor Shield logic | 5 V | 0.1 A |
| HLS-LFCD2 | 5 V | 0.5 A |
| IMU boards | 3.3 V | 0.05 A |
| **Total** | | **~6.5 A peak** |

A **3S LiPo 2200 mAh** provides roughly **25–30 min** run time at average load.
