# force-sensor

Two-jaw robotic gripper with closed-loop force feedback on each fingertip. An Arduino Nano reads an FSR at each tip and drives each jaw's servo independently, so the gripper closes around off-center objects without crushing them: whichever tip touches first stops, and the other jaw keeps closing until it also reads the target force.

Built as a portfolio piece for the embedded / control side. The mechanism is intentionally generic — any pinch-style gripper with one servo per jaw will work.

## Bill of materials

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino Nano (ATmega328P, 5V) | Any 5V AVR Arduino works; for 3.3V boards change `VREF` in the sketch. |
| 2 | Interlink FSR-402 | Round force-sensing resistor, ~0.2 N to 20 N usable range. |
| 2 | 10 kΩ resistor, 1/4 W | Pulldown for each FSR voltage divider. |
| 2 | Hobby servo (SG90, MG90S, MG996R, etc.) | One per jaw. Size to your linkage. |
| 1 | External 5–6 V supply for the servos | 1 A is plenty for two micro servos; more for metal-gear. |
| — | Hookup wire, breadboard or perfboard | |

Do not power the servos from the Nano's 5 V pin. Two servos stalling on a jammed object will brown the Nano out mid-grip and you'll lose the control loop right when you need it. Run them off the external supply and tie grounds together.

## Wiring

FSR voltage divider, one per tip:

```
        5V ────┬──── FSR ────┬──── A0  (or A1 for the other tip)
               │             │
                             └──── 10kΩ ──── GND
```

Servos:

| Pin | Connection |
|---|---|
| D9  | Left jaw servo signal |
| D10 | Right jaw servo signal |
| A0  | Left tip FSR divider tap |
| A1  | Right tip FSR divider tap |
| 5–6 V ext. | Servo V+ |
| GND | Servo GND + Nano GND (common) |

D9 and D10 are both PWM-capable on the Nano, which the `Servo` library needs.

## Control approach

Each jaw runs its own independent loop at 50 Hz. Per tick, for each jaw:

1. Read the FSR ADC eight times, average, subtract the no-load baseline captured at startup (`z` tares it manually).
2. Convert ADC → resistance via the divider equation, then → force using the FSR-402 datasheet's conductance-vs-force curve. Two-segment fit, continuous at the 1 mS knee.
3. If force is below `TARGET_FORCE_N - deadband`, step the jaw 1° toward closed. If above `+deadband`, step 1° toward open. Otherwise hold. The deadband (0.15 N default) kills oscillation around the setpoint.
4. Hard safety: if either tip exceeds `MAX_FORCE_N` (8 N default), back that jaw off immediately and drop to idle.

Decoupling the two jaws is the whole point. A single global controller that averaged the two tips would over-close on whichever side wasn't in contact yet. Running them independently means an asymmetric object — a pen held off-center, a screw cap, a finger — gets gripped evenly without either side overshooting.

The FSR conversion (`adcToForceN` in [robotic_claw.ino](robotic_claw.ino)) is the standard Adafruit/Interlink piecewise curve, with one correction: the upper branch is offset by +12.5 N so the two segments meet at the 1 mS knee instead of jumping discontinuously. Good enough for relative-force control; calibrate against known weights if you need absolute accuracy better than ±20%.

## Use

Flash [robotic_claw.ino](robotic_claw.ino), open the serial monitor at 115200 baud, and:

| Command | Effect |
|---|---|
| `o`     | Open both jaws |
| `c`     | Close to target force (per-jaw) |
| `t2.5`  | Set target force to 2.5 N |
| `z`     | Re-tare FSR baselines (jaws must be open and untouched) |

Telemetry streams while running:

```
L: 1.94 N @ 112deg   R: 2.07 N @ 48deg   mode=close
```

## Tuning before first run

Things in the sketch worth setting for your specific build:

- `LEFT_OPEN_DEG` / `LEFT_CLOSED_DEG` and `RIGHT_OPEN_DEG` / `RIGHT_CLOSED_DEG` — the servo angles where each jaw is fully open and fully closed. Get these right before powering up so a runaway loop can't grind the linkage.
- `TARGET_FORCE_N` — default 2 N. Comfortable for picking up a pen or a small bottle.
- `MAX_FORCE_N` — default 8 N. Below the FSR-402's saturation point and well under what an MG996R can deliver, so the safety stop actually triggers before the mechanism stalls.
- `STEP_DEG` — increase for faster closing, decrease if the loop overshoots.
