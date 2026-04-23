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

const double HEIGHT_THRESHOLD = 3.0;  // meters
const float MAX_FALLING_SPEED = 5.0;  // m/s safety threshold

// --- Button Debounce ---
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 200;
const unsigned long LOOP_DELAY = 50;  // milliseconds

// --- Timing for Speed Calculation ---
unsigned long lastSensorReadTime = 0;
const unsigned long SPEED_CALC_INTERVAL = 50;  // Calculate speed every 50ms

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
}

void loop() {
  handleButton();
  readSensors();
  calculateFallingSpeed();
  logData();

  if (systemArmed && currentHeight > HEIGHT_THRESHOLD && !triggerExecuted) {
    executeTrigger();
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
      triggerExecuted = false;  // Reset trigger flag
      
      Serial.println("\n*** SYSTEM ARMED ***");
      playArmingTone();
      
      lastButtonPress = currentTime;
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

void calculateFallingSpeed() {
  unsigned long currentTime = millis();
  unsigned long timeDelta = currentTime - lastSensorReadTime;
  
  if (timeDelta >= SPEED_CALC_INTERVAL) {
    float heightDelta = currentHeight - previousHeight;  // Negative when falling
    float timeDeltaSeconds = timeDelta / 1000.0;
    
    fallingSpeed = (heightDelta / timeDeltaSeconds);  // m/s (negative = falling)
    
    lastSensorReadTime = currentTime;
  }
}

void logData() {
  Serial.print("[");
  Serial.print(millis() / 1000);  // Timestamp in seconds
  Serial.print("s] Height: ");
  Serial.print(currentHeight, 2);
  Serial.print(" m | Speed: ");
  Serial.print(fallingSpeed, 2);
  Serial.print(" m/s | Distance: ");
  Serial.print(currentDistance);
  Serial.print(" cm | Armed: ");
  Serial.println(systemArmed ? "YES" : "NO");
}

void executeTrigger() {
  Serial.println("\n*** TRIGGER ACTIVATED ***");
  Serial.print("Capture Height: ");
  Serial.print(currentHeight, 2);
  Serial.print(" m | Falling Speed: ");
  Serial.print(fallingSpeed, 2);
  Serial.println(" m/s");

  // Activate servo trigger
  cameraShutter.write(70);
  delay(300);
  cameraShutter.write(90);

  playTriggerTone();

  triggerExecuted = true;
  systemArmed = false;
  digitalWrite(PIN_LED, LOW);
  
  Serial.println("*** TRIGGER COMPLETE ***\n");
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
