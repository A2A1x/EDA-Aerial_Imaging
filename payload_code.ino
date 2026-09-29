/*
  README payload summary (embedded)

  Purpose:
  - Monitor payload altitude and proximity for an aerial imaging system.
  - Smooth altitude readings with a simple Kalman-style filter, estimate vertical speed,
    and trigger a servo-controlled camera shutter under configurable conditions.
  - Provide audio (piezo) and visual (LED) feedback and Serial telemetry.

  Required hardware & libraries:
  - Arduino-compatible board
  - Servo connected to PIN_SERVO (pin 3)
  - Altimeter using the EYW_alt library (object: altitude)
  - Ultrasonic distance sensor HCSR04 (trig: PIN_TRIG=6, echo: PIN_ECHO=7)
  - Piezo speaker on PIN_SPEAKER (pin 5)
  - Arm/disarm button on PIN_BUTTON (pin 2) - each press toggles
  - Status LED on PIN_LED (pin 4) and onboard LED on pin 13
  - Libraries: Servo, HCSR04, EYW_alt

  High-level behavior:
  1) setup(): initialize Serial, pins, attach servo, calibrate altimeter, play startup tone.
  2) loop(): handle button press (arming), read sensors, smooth height (Kalman),
     calculate falling speed, check ground impact.
  3) Triggers:
     - Initial trigger: when armed and kalmanHeight > HEIGHT_THRESHOLD (default 3.0 m),
       actuate servo to mimic camera shutter and play a trigger tone (one-time per arm).
     - Periodic trigger: while armed and airborne, actuate servo every
       PERIODIC_TRIGGER_INTERVAL (default 3 seconds) and play a periodic tone.
  4) Ground impact (only checked after the initial trigger, so arming on the ground
     doesn't instantly disarm): uses the ultrasonic proximity sensor (HCSR04). If the measured
     distance to the ground (converted to meters) is <= GROUND_IMPACT_HEIGHT (0.1 m)
     while armed, flag ground impact, auto-disarm, turn off LED and play landing tone.
     If the ultrasonic sensor returns an invalid reading (0), the code falls back to
     the altimeter kalmanHeight check.

  Key configuration defaults (see constants in the sketch):
  - HEIGHT_THRESHOLD = 3.0 m
  - PERIODIC_TRIGGER_INTERVAL = 3000 ms (3 s)
  - GROUND_IMPACT_HEIGHT = 0.1 m
  - SPEED_CALC_INTERVAL = 50 ms
  - DEBOUNCE_DELAY = 200 ms

  Notes / suggestions:
  - The Kalman-like filter here is simple; tune kalmanEstimateError and
    kalmanMeasurementError for your altimeter noise characteristics.
  - The sketch uses blocking delay() calls for servo timing and tones; for
    improved responsiveness convert those to millis()-based non-blocking timers.
  - MAX_FALLING_SPEED is defined but unused; consider checking it before
    actuating the servo to avoid triggering during fast descent.
  - To test on bench: simulate arming and mock or manually vary altitude and
    proximity readings while observing Serial output.
*/

#include <Servo.h>
#include <HCSR04.h>
#include <EYW_alt.h>

// ============================================================================
// PAYLOAD CODE - Aerial Imaging System with Altitude & Speed Monitoring
// ============================================================================

// --- Pin Definitions ---
const int PIN_LED = 4;
const int PIN_SPEAKER = 5;
const int PIN_BUTTON = 2;
const int PIN_SERVO = 3;
const int PIN_TRIG = 6;
const int PIN_ECHO = 7;
const int PIN_ONBOARD_LED = 13;

// --- Objects ---
EYW_alt::Altimeter altitude;
UltraSonicDistanceSensor proximity(PIN_TRIG, PIN_ECHO);
Servo cameraShutter;

// --- State Variables ---
float currentHeight = 0;
float fallingSpeed = 0;  // meters per second
int currentDistance = 0;

bool systemArmed = false;
bool triggerExecuted = false;
bool groundImpactDetected = false;

const double HEIGHT_THRESHOLD = 3.0;      // meters
const float MAX_FALLING_SPEED = 5.0;      // m/s safety threshold
const double GROUND_IMPACT_HEIGHT = 0.1;  // meters - consider landed if below this
const unsigned long PERIODIC_TRIGGER_INTERVAL = 3000;  // 3 seconds in milliseconds

// --- Button Debounce ---
unsigned long lastButtonPress = 0;
int lastButtonState = LOW;
const unsigned long DEBOUNCE_DELAY = 200;
const unsigned long LOOP_DELAY = 50;  // milliseconds

// --- Timing for Speed Calculation ---
unsigned long lastSensorReadTime = 0;
float lastSpeedHeight = 0;  // kalmanHeight at last speed calculation
const unsigned long SPEED_CALC_INTERVAL = 50;  // Calculate speed every 50ms

// --- Timing for Periodic Servo Actuation ---
unsigned long lastPeriodicTriggerTime = 0;
unsigned int shotCount = 0;

// --- Kalman Filter Variables ---
float kalmanHeight = 0;
float kalmanEstimate = 0;
float kalmanEstimateError = 0.1;
float kalmanMeasurementError = 0.2;
float kalmanProcessNoise = 0.1;  // higher = trusts new readings more (0.1 -> steady gain 0.5)
float kalmanGain = 0;

void setup() {
  Serial.begin(9600);
  delay(100);

  // Initialize pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_ONBOARD_LED, OUTPUT);

  // Initialize servo
  cameraShutter.attach(PIN_SERVO);
  cameraShutter.write(90);  // Neutral position

  // Calibrate altimeter
  Serial.println("========================================");
  Serial.println("AERIAL IMAGING SYSTEM - INITIALIZING");
  Serial.println("========================================");
  Serial.println("Calibrating Altimeter... Keep steady.");
  
  altitude.begin();
  altitude.calibrate(100);
  
  Serial.println("Calibration complete.");
  Serial.println("System ready.");
  Serial.println("========================================");

  // Startup signal
  playStartupTone();

  lastSensorReadTime = millis();
  lastPeriodicTriggerTime = millis();
}

void loop() {
  handleButton();
  readSensors();
  applyKalmanFilter();
  calculateFallingSpeed();
  checkGroundImpact();
  logData();

  // Trigger on initial height threshold (manual trigger)
  if (systemArmed && kalmanHeight > HEIGHT_THRESHOLD && !triggerExecuted) {
    executeTrigger();
    lastPeriodicTriggerTime = millis();  // Reset periodic timer
    shotCount = 0;
  }

  // Periodic servo actuation while armed and airborne
  if (systemArmed && kalmanHeight > GROUND_IMPACT_HEIGHT && !groundImpactDetected) {
    unsigned long currentTime = millis();
    if (currentTime - lastPeriodicTriggerTime >= PERIODIC_TRIGGER_INTERVAL) {
      executePeriodicTrigger();
      lastPeriodicTriggerTime = currentTime;
    }
  }

  // Auto-disarm on ground impact
  if (groundImpactDetected && systemArmed) {
    systemArmed = false;
    digitalWrite(PIN_LED, LOW);
    playLandingTone();
    Serial.println("\n*** GROUND IMPACT DETECTED - SYSTEM DISARMED ***\n");
  }

  delay(LOOP_DELAY);
}

// ============================================================================
// FUNCTION DEFINITIONS
// ============================================================================

void handleButton() {
  int state = digitalRead(PIN_BUTTON);
  unsigned long currentTime = millis();

  // Act on the press edge only, so holding the button doesn't re-arm repeatedly
  if (state == HIGH && lastButtonState == LOW && currentTime - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = currentTime;
    systemArmed = !systemArmed;
    digitalWrite(PIN_LED, systemArmed ? HIGH : LOW);

    if (systemArmed) {
      triggerExecuted = false;
      groundImpactDetected = false;
      shotCount = 0;
      lastPeriodicTriggerTime = currentTime;
      Serial.println("\n*** SYSTEM ARMED ***");
      playArmingTone();
    } else {
      Serial.println("\n*** SYSTEM DISARMED ***");
      playLandingTone();
    }
  }
  lastButtonState = state;
}

void readSensors() {
  currentHeight = altitude.getHeightAvg(20);  // 20-sample average
  currentDistance = proximity.measureDistanceCm();
}

void applyKalmanFilter() {
  // Predict: uncertainty grows by process noise
  kalmanEstimateError += kalmanProcessNoise;
  kalmanGain = kalmanEstimateError / (kalmanEstimateError + kalmanMeasurementError);
  
  // Update estimate
  kalmanEstimate = kalmanEstimate + kalmanGain * (currentHeight - kalmanEstimate);
  
  // Update error estimate
  kalmanEstimateError = (1 - kalmanGain) * kalmanEstimateError;
  
  kalmanHeight = kalmanEstimate;
}

void calculateFallingSpeed() {
  unsigned long currentTime = millis();
  unsigned long timeDelta = currentTime - lastSensorReadTime;
  
  if (timeDelta >= SPEED_CALC_INTERVAL) {
    float heightDelta = kalmanHeight - lastSpeedHeight;  // Negative when falling
    float timeDeltaSeconds = timeDelta / 1000.0;
    
    fallingSpeed = (heightDelta / timeDeltaSeconds);  // m/s (negative = falling)
    
    lastSensorReadTime = currentTime;
    lastSpeedHeight = kalmanHeight;
  }
}

void checkGroundImpact() {
  // Only after the initial trigger (i.e. we've been above HEIGHT_THRESHOLD);
  // otherwise arming while sitting on the ground disarms on the next loop.
  if (!triggerExecuted) return;

  // Use ultrasonic proximity sensor to detect ground impact.
  // currentDistance is in cm; convert to meters.
  // Many HCSR04 libraries return 0 when out-of-range / invalid, so treat 0 as invalid.
  if (currentDistance > 0) {
    float distanceMeters = currentDistance / 100.0;
    if (distanceMeters <= GROUND_IMPACT_HEIGHT && systemArmed) {
      groundImpactDetected = true;
    }
  } else {
    // Fallback to altimeter-based detection if ultrasonic reading is invalid
    if (kalmanHeight <= GROUND_IMPACT_HEIGHT && systemArmed) {
      groundImpactDetected = true;
    }
  }
}

void logData() {
  Serial.print("[");
  Serial.print(millis() / 1000);  // Timestamp in seconds
  Serial.print("s] Height: ");
  Serial.print(kalmanHeight, 2);
  Serial.print(" m | Raw: ");
  Serial.print(currentHeight, 2);
  Serial.print(" m | Speed: ");
  Serial.print(fallingSpeed, 2);
  Serial.print(" m/s | Distance: ");
  Serial.print(currentDistance);
  Serial.print(" cm | Armed: ");
  Serial.println(systemArmed ? "YES" : "NO");
}

void executeTrigger() {
  Serial.println("\n*** INITIAL TRIGGER ACTIVATED ***");
  Serial.print("Capture Height: ");
  Serial.print(kalmanHeight, 2);
  Serial.print(" m | Falling Speed: ");
  Serial.print(fallingSpeed, 2);
  Serial.println(" m/s");

  // Activate servo trigger
  cameraShutter.write(70);
  delay(300);
  cameraShutter.write(90);

  playTriggerTone();

  triggerExecuted = true;
  
  Serial.println("*** INITIAL TRIGGER COMPLETE ***\n");
}

void executePeriodicTrigger() {
  shotCount++;
  
  Serial.println("\n*** PERIODIC SERVO ACTUATION ***");
  Serial.print("  >> Actuation #");
  Serial.print(shotCount);
  Serial.print(" | Height: ");
  Serial.print(kalmanHeight, 2);
  Serial.print(" m | Speed: ");
  Serial.print(fallingSpeed, 2);
  Serial.println(" m/s");

  // Activate servo
  cameraShutter.write(70);
  delay(300);
  cameraShutter.write(90);

  playPeriodicTone();
}

void playStartupTone() {
  tone(PIN_SPEAKER, 800, 200);
  delay(250);
  tone(PIN_SPEAKER, 1200, 200);
  digitalWrite(PIN_LED, HIGH);
  delay(250);
  digitalWrite(PIN_LED, LOW);
}

void playArmingTone() {
  tone(PIN_SPEAKER, 600, 150);
  delay(200);
  tone(PIN_SPEAKER, 900, 150);
}

void playTriggerTone() {
  tone(PIN_SPEAKER, 1200, 100);
  delay(150);
  tone(PIN_SPEAKER, 1200, 100);
  delay(150);
  tone(PIN_SPEAKER, 1200, 200);
}

void playPeriodicTone() {
  tone(PIN_SPEAKER, 1000, 150);
  delay(200);
  tone(PIN_SPEAKER, 1000, 150);
}

void playLandingTone() {
  tone(PIN_SPEAKER, 600, 200);
  delay(250);
  tone(PIN_SPEAKER, 400, 200);
  delay(250);
  tone(PIN_SPEAKER, 200, 300);
}
