# Motor Encoder PID Control

A bare-metal PID position controller for an ESP32 + L298N motor driver + TT motor with quadrature encoder. The PID algorithm is written directly in the sketch (no library) so the math is fully visible and tunable from the Serial Monitor.

-----

## Download

1. Click the green **`<> Code`** button at the top of this repository
1. Select **Download ZIP**
1. Extract the ZIP to your Arduino sketchbook folder
- Windows: `Documents\Arduino\`
- Mac/Linux: `~/Arduino/`
1. Open **`TT_Motor_PID_Control.ino`** in Arduino IDE

> **Board package required:** ESP32 by Espressif  
> Install via: `File > Preferences > Additional Boards Manager URLs`  
> Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`  
> Then: `Tools > Board > Boards Manager` → search **esp32** → Install

-----

## Hardware

### Components

- ESP32 Dev Module
- L298N Motor Driver Module
- TT Gear Motor with Quadrature Encoder
- 6–9V external power supply (for motor)
- USB cable (ESP32 to PC)

### Pin Assignments

|Signal     |ESP32 GPIO|L298N / Encoder                      |
|-----------|----------|-------------------------------------|
|IN1        |GPIO 26   |L298N IN1 — motor direction          |
|IN2        |GPIO 25   |L298N IN2 — motor direction          |
|ENA        |GPIO 27   |L298N ENA — PWM speed control        |
|ENC A      |GPIO 32   |Encoder channel A                    |
|ENC B      |GPIO 33   |Encoder channel B                    |
|GND        |GND       |Common ground — ESP32, L298N, encoder|
|5V         |5V or Vin |Encoder VCC logic power              |
|Motor power|—         |6–9V to L298N VS terminal            |

### Wiring Diagram

![Wiring Diagram](wiring_diagram.jpg)

-----

## Testing the Code

### 1. Upload

- In Arduino IDE: `Tools > Board` → **ESP32 Dev Module**
- `Tools > Port` → select your ESP32 COM port
- Click **Upload** (→ arrow button)
- Wait for *Done uploading*

### 2. Open Serial Monitor

- `Tools > Serial Monitor`
- Set baud rate to **115200**
- Set line ending to **Newline**

You should see the startup message and the column header `Encoder, Command` printed once.

### 3. Verify Encoder

Before running the motor, rotate the shaft by hand. Watch the `Encoder:` value in Serial Monitor — it should **increase** in one direction and **decrease** in the other. If it only counts up regardless of direction, swap the ENC A and ENC B wires.

### 4. Serial Commands

Send commands by typing in the Serial Monitor input box and pressing Enter.

|Command   |Example|Action                     |
|----------|-------|---------------------------|
|`a`       |`a`    |Step to setpoint 500 counts|
|`z`       |`z`    |Return to zero (home)      |
|`s<value>`|`s750` |Set a custom setpoint      |
|`p<value>`|`p0.5` |Set Kp gain                |
|`i<value>`|`i0.01`|Set Ki gain                |
|`d<value>`|`d0.05`|Set Kd gain                |
|`r`       |`r`    |Reset encoder count to zero|

### 5. Basic Tuning Sequence

Start with all gains at zero and work up one term at a time:

**Step 1 — P only**

```
p0.3
i0.0
d0.0
a
```

Send `a` to command 500 counts. The motor should move toward the target. If it doesn’t move, increase Kp. If it oscillates, decrease Kp.

**Step 2 — Add D to reduce overshoot**

```
d0.05
a
```

Increase Kd gradually until overshoot is damped. Too much Kd causes jitter.

**Step 3 — Add I to eliminate steady-state error**

```
i0.01
a
```

Increase Ki slowly. If the motor oscillates, reduce Ki. The integral term will push the motor to exactly zero error over time.

**Step 4 — Test return to home**

```
z
```

Motor should return to zero counts. Tune until both directions respond cleanly.

### 6. Serial Plotter (optional)

Close Serial Monitor, then open `Tools > Serial Plotter`. Send `a` to command a step. You will see live traces for encoder position and PID output — useful for visualizing overshoot and settling time.

-----

## PID Algorithm

The controller is implemented in `computePID()` with no external library. Each term is a separate, labeled line of code:

```
error      = setpoint - input
P          = Kp * error
integral  += error * dt          (clamped to prevent windup)
I          = Ki * integral
derivative = (error - lastError) / dt
D          = Kd * derivative
output     = P + I + D           (clamped to -255 … +255)
```

Gains can be changed live over Serial without re-uploading.

-----

## Troubleshooting

|Symptom                                  |Likely cause                |Fix                                           |
|-----------------------------------------|----------------------------|----------------------------------------------|
|Motor doesn’t move                       |Kp too low, deadband, wiring|Increase Kp; check IN1/IN2 and ENA connections|
|Motor runs full speed, won’t stop        |Encoder not reading         |Check ENC A/B wiring and pullup pins          |
|Motor oscillates continuously            |Kp too high                 |Reduce Kp, add Kd                             |
|Steady-state error (never reaches target)|Ki = 0                      |Add small Ki value                            |
|Encoder counts wrong direction           |A/B channels swapped        |Swap ENC A and ENC B wires                    |
|Upload fails                             |Wrong COM port or board     |Check Tools > Board and Tools > Port          |