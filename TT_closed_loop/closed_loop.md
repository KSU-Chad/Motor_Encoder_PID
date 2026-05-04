# Activity 2 — Closed Loop Velocity PID Control
### MET 382 Industrial Instrumentation and Controls — KSU Salina

PID controller maintains constant motor speed under varying mechanical load. The controller continuously measures encoder velocity and adjusts PWM output to eliminate the difference between the target speed and actual speed.

> **Complete Activity 1 first.** You will need your measured no-load velocity from Activity 1 to set the correct setpoint here.

---

## Download

1. Click the green **`<> Code`** button at the top of this repository
2. Select **Download ZIP**
3. Extract the ZIP to your Arduino sketchbook folder
   - Windows: `Documents\Arduino\`
   - Mac/Linux: `~/Arduino/`
4. Open **`Velocity_PID.ino`** in Arduino IDE

> **Board package required:** ESP32 by Espressif
> Install via: `File > Preferences > Additional Boards Manager URLs`
> Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
> Then: `Tools > Board > Boards Manager` → search **esp32** → Install

---

## Hardware

### Pin Assignments

| Signal | ESP32 GPIO | Connected To |
|--------|-----------|--------------|
| IN1 | GPIO 26 | L298N IN1 — motor direction |
| IN2 | GPIO 25 | L298N IN2 — motor direction |
| ENA | GPIO 27 | L298N ENA — PWM speed control |
| ENC A | GPIO 32 | Encoder channel A |
| ENC B | GPIO 33 | Encoder channel B |
| GND | GND | Common ground — ESP32, L298N, encoder |
| 5V | 5V or Vin | Encoder VCC logic power |
| Motor power | External | 6–9V to L298N VS terminal |

### Wiring Diagram

<!-- Add wiring diagram image here -->

---

## How the PID Algorithm Works

The controller runs every 100 ms. Each cycle it:

1. Measures velocity — encoder counts since the last cycle, divided by elapsed time
2. Calculates error — `error = setpoint − velocity`
3. Computes PID output:

```
P = Kp × error
I = Ki × (sum of all past errors × dt)       ← does the heavy lifting
D = Kd × (change in error ÷ dt)              ← leave at 0 for velocity control
output = P + I + D                            ← sent to motor as PWM duty
```

**Why Ki matters most for velocity control:**
If the motor is running slightly too slow, the error is small but persistent. Kp alone gives a small correction that may not be enough. Ki accumulates the error over time and keeps increasing the PWM output until the speed actually matches the setpoint — even under load.

**Why Kd stays at zero:**
Velocity is already the rate of change of position. Taking the derivative of a velocity measurement amplifies encoder noise and makes the output erratic. Leave Kd at 0.

---

## Running the Sketch

### 1. Upload

- `Tools > Board` → **ESP32 Dev Module**
- `Tools > Port` → select your ESP32 COM port
- Click **Upload**

### 2. Set the Velocity Setpoint

Use 80% of your no-load velocity from Activity 1. This gives the PID headroom to increase PWM when load is applied.

**Example:** No-load speed = 900 counts/sec → set setpoint to 720

Open `Tools > Serial Monitor` at 115200 baud. Send the command:

```
v720
```

Replace 720 with your calculated value. The controller will immediately begin targeting that speed.

### 3. Open Serial Plotter

- Close Serial Monitor
- `Tools > Serial Plotter`
- Set baud rate to **115200**

You will see two traces:
- `Setpoint` — your target velocity (flat line)
- `Velocity` — actual measured speed

The PID controller should hold the velocity trace close to the setpoint line.

### 4. Apply Load

Press gently on the motor shaft with your finger. Hold for 5 seconds, then release. Compare to what you saw in Activity 1 — the PID controller should increase PWM automatically to compensate and keep the velocity trace near the setpoint.

### 5. Test the Limit

Increase load until the controller can no longer hold the setpoint. At this point the PWM output is at 255 (fully saturated) and cannot go higher. The velocity will drop and stay below the setpoint. This is the physical limit of the system.

---

## Serial Commands

Send commands via `Tools > Serial Monitor` at 115200 baud with line ending set to **Newline**. Close Serial Monitor before opening Serial Plotter.

| Command | Example | Action |
|---------|---------|--------|
| `v<value>` | `v720` | Set velocity setpoint (counts/sec) |
| `p<value>` | `p0.1` | Set Kp gain |
| `i<value>` | `i0.05` | Set Ki gain (resets integral) |
| `d<value>` | `d0.0` | Set Kd gain (keep at 0) |
| `x` | `x` | Stop motor |

---

## Tuning Guide

Start with the default gains. Tune one at a time in this order:

**Step 1 — Kp (default: 0.1)**
```
p0.1
```
If velocity oscillates above and below the setpoint, reduce Kp. If response is sluggish, increase slightly. Kp gives the initial fast reaction.

**Step 2 — Ki (default: 0.05)**
```
i0.05
```
Ki drives the velocity to exactly match the setpoint. Increase slowly — too much causes a slow growing oscillation (integral windup). If windup occurs, send `x` to stop, reduce Ki, and restart.

**Step 3 — Kd (leave at 0)**
```
d0.0
```
Do not increase Kd for velocity control. Velocity measurement is inherently noisy and Kd amplifies that noise into erratic output.

### Starting Point Reference

| Gain | Default | Typical range for TT motors |
|------|---------|----------------------------|
| Kp | 0.1 | 0.05 – 0.5 |
| Ki | 0.05 | 0.01 – 0.2 |
| Kd | 0.0 | always 0 |

---

## What to Observe

| Condition | Expected behavior |
|-----------|------------------|
| No load, PID running | Velocity trace holds near setpoint |
| Light load applied | Brief dip, PID corrects within 1–2 seconds |
| Heavy load applied | Larger dip, PID corrects but takes longer |
| Load exceeds motor capability | Velocity stays below setpoint — PWM at 255, PID saturated |
| Load released | Velocity briefly overshoots, then settles back to setpoint |

---

## Comparing to Activity 1

| | Activity 1 — Open Loop | Activity 2 — Velocity PID |
|--|------------------------|--------------------------|
| PWM output | Fixed | Adjusts automatically |
| Response to load | Speed drops, stays low | Speed dips, corrects |
| Steady-state error | Permanent while loaded | Eliminated by Ki |
| Key limitation | No feedback | Motor/driver power ceiling |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motor does not spin at startup | Integral not seeded, Kp/Ki too low | Send `v<setpoint>` to start; increase Ki slightly |
| Velocity oscillates constantly | Kp or Ki too high | Halve both values; retune from scratch |
| Velocity never reaches setpoint | Setpoint too high for motor | Reduce setpoint to 70–80% of no-load speed |
| Plotter shows one flat line | Baud rate mismatch | Set Serial Plotter to 115200 |
| Motor runs away at full speed | Encoder not reading | Check ENC A/B wiring — PID sees zero velocity and winds up |
