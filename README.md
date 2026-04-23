# EDA-Aerial_Imaging

## payload_code.ino — Aerial payload firmware (overview)

This repository includes `payload_code.ino`, an Arduino/embedded firmware sketch intended for an aerial imaging payload. The file implements altitude monitoring, a simple Kalman filter for smoothing height measurements, fall-speed estimation, and servo-based camera triggering with audio/LED cues. Below is a concise explanation of what the sketch does and how it works.

### Purpose

- Monitor payload altitude and proximity while airborne.
- Automatically trigger a camera shutter (via a servo) under two conditions:
  - An initial trigger when the system is armed and a configurable height threshold is exceeded (manual/mission start).
  - Periodic servo actuations (every 30 seconds) while the system remains armed and airborne.
- Detect ground impact and automatically disarm the system.
- Provide auditory (speaker tones) and visual (LED) feedback for state changes.
- Print runtime telemetry to the Serial console for logging and debugging.

### Required hardware & libraries

- Arduino-compatible board (pins are defined in the sketch)
- Servo (camera shutter) attached to PIN_SERVO (pin 3)
- Altimeter using the `EYW_alt` library (object `altitude`)
- Ultrasonic distance sensor HCSR04 (trig pin 6, echo pin 7) using `HCSR04` library
- Piezo speaker on PIN_SPEAKER (pin 5)
- Arm/disarm button on PIN_BUTTON (pin 2)
- Status LED on PIN_LED (pin 4) and onboard LED on pin 13

Libraries used in the sketch:
- Servo
- HCSR04
- EYW_alt

Make sure these libraries are installed in your Arduino environment before compiling.

### High-level behavior

1. Initialization (setup):
   - Serial at 9600 baud for telemetry.
   - Pins are configured and the servo is attached and moved to a neutral position (90°).
   - The altimeter is initialized and calibrated (the sketch calls `altitude.calibrate(100)`).
   - A startup tone and LED blink indicate readiness.

2. Arming (handleButton):
   - Pressing the configured button (debounced) arms the system.
   - When armed: LED turns on, arming tone plays, `systemArmed` is set true, and periodic timers reset.

3. Sensor reading (readSensors):
   - The altimeter returns a 20-sample averaged height (meters).
   - The ultrasonic sensor returns a distance measurement in centimeters.

4. Kalman filter (applyKalmanFilter):
   - A simple 1D Kalman-like update smooths the height measurement and stores the result in `kalmanHeight`. This reduces noise from the altimeter readings before downstream logic uses the height.

5. Falling speed calculation (calculateFallingSpeed):
   - Every ~50 ms the sketch computes vertical speed (m/s) from the change in the Kalman-smoothed height.
   - Negative values indicate descent (falling).

6. Trigger logic
   - Initial trigger (executeTrigger): When the system is armed and `kalmanHeight` exceeds `HEIGHT_THRESHOLD` (default 3.0 m) and the initial trigger hasn't run yet, the code activates the servo to mimic a camera shutter (moves to 70°, waits 300 ms, returns to 90°), plays a trigger tone, and sets `triggerExecuted`.
   - Periodic trigger (executePeriodicTrigger): While still armed and above the ground impact threshold, the code will actuate the servo every `PERIODIC_TRIGGER_INTERVAL` (default 30 seconds). Each actuation increments `shotCount` and plays a periodic tone.

7. Ground impact & safety
   - If `kalmanHeight` falls below or equal to `GROUND_IMPACT_HEIGHT` (0.1 m) while the system is armed, the sketch flags a ground impact (`groundImpactDetected`) and automatically disarms the system (LED off, landing tone, and a Serial message).
   - Note: `MAX_FALLING_SPEED` is defined (5.0 m/s) as a safety threshold but is not used in the current logic; you may extend the sketch to check `fallingSpeed` against this value for additional safety behavior.

8. Telemetry & feedback
   - The sketch prints periodic telemetry lines to Serial with timestamp (seconds), kalman-smoothed height, raw height, computed speed, ultrasonic distance, and armed state.
   - Audio cues (via `tone()` on the speaker pin) signal startup, arming, triggers, periodic shots, and landing.
   - LED indicates armed state when the main status LED is lit.

### Key configuration values (defaults in the sketch)

- HEIGHT_THRESHOLD = 3.0 m — initial trigger height
- PERIODIC_TRIGGER_INTERVAL = 30000 ms — 30s between periodic actuations
- GROUND_IMPACT_HEIGHT = 0.1 m — threshold to consider payload landed
- SPEED_CALC_INTERVAL = 50 ms — how often falling speed is recalculated
- DEBOUNCE_DELAY = 200 ms — button debounce window

### Notes & suggestions

- The Kalman filter implementation here is a simple form and may need tuning for your specific altimeter sensor noise characteristics (adjust `kalmanEstimateError` and `kalmanMeasurementError`).
- Delays (`delay()`) are used for servo timing and tones; these block the main loop and may affect responsiveness. If you need non-blocking behavior, consider refactoring to use millis()-based timers instead of `delay()`.
- `MAX_FALLING_SPEED` is present but unused: you can add a check in the trigger and periodic logic to avoid actuating when falling too quickly.
- To test without flight, attach the hardware on a bench and simulate arming + changing the altitude input (or mock the altitude readings) while observing Serial output.

## License

This repository is provided under the MIT License. See LICENSE for details (or add one if missing).