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
float previousHeight = 0;
float fallingSpeed = 0;  // meters per second
int currentDistance = 0;

bool systemArmed = false;
bool triggerExecuted = false;
bool groundImpactDetected = false;

const double HEIGHT_THRESHOLD = 3.0;      // meters
const float MAX_FALLING_SPEED = 5.0;      // m/s safety threshold
const double GROUND_IMPACT_HEIGHT = 0.1;  // meters - consider landed if below this
const unsigned long PERIODIC_TRIGGER_INTERVAL = 30000;  // 30 seconds in milliseconds

// --- Button Debounce ---
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 200;
const unsigned long LOOP_DELAY = 50;  // milliseconds

// --- Timing for Speed Calculation ---
unsigned long lastSensorReadTime = 0;
const unsigned long SPEED_CALC_INTERVAL = 50;  // Calculate speed every 50ms

// --- Timing for Periodic Servo Actuation ---
unsigned long lastPeriodicTriggerTime = 0;
unsigned int shotCount = 0;

// --- Kalman Filter Variables ---
float kalmanHeight = 0;
float kalmanEstimate = 0;
float kalmanEstimateError = 0.1;
float kalmanMeasurementError = 0.2;
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

  // Periodic servo actuation every 30 seconds while armed and airborne
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
  if (digitalRead(PIN_BUTTON) == HIGH) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastButtonPress > DEBOUNCE_DELAY) {
      digitalWrite(PIN_LED, HIGH);
      systemArmed = true;
      triggerExecuted = false;
      groundImpactDetected = false;
      shotCount = 0;
      
      Serial.println("\n*** SYSTEM ARMED ***");
      playArmingTone();
      
      lastButtonPress = currentTime;
      lastPeriodicTriggerTime = currentTime;
    }
  } else {
    digitalWrite(PIN_LED, LOW);
  }
}

void readSensors() {
  previousHeight = currentHeight;
  currentHeight = altitude.getHeightAvg(20);  // 20-sample average
  currentDistance = proximity.measureDistanceCm();
}

void applyKalmanFilter() {
  // Kalman gain calculation
  kalmanEstimateError += kalmanEstimateError;
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
    float heightDelta = kalmanHeight - previousHeight;  // Negative when falling
    float timeDeltaSeconds = timeDelta / 1000.0;
    
    fallingSpeed = (heightDelta / timeDeltaSeconds);  // m/s (negative = falling)
    
    lastSensorReadTime = currentTime;
  }
}

void checkGroundImpact() {
  if (kalmanHeight <= GROUND_IMPACT_HEIGHT && systemArmed) {
    groundImpactDetected = true;
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
