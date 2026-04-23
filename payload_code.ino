#include <Servo.h>
#include <HCSR04.h>
#include <EYW_alt.h>

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
int currentDistance = 0;

bool systemArmed = false;
bool triggerExecuted = false;

double heightTresshold = 3;

// Button debounce
unsigned long lastButtonPress = 0;
const int debounceDelay = 200;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_ONBOARD_LED, OUTPUT);

  cameraShutter.attach(PIN_SERVO);
  cameraShutter.write(90);

  Serial.println("Calibrating Altimeter... Keep steady.");
  altitude.begin();
  altitude.calibrate(100);
  Serial.println("Calibration complete.");

  // Startup signal
  tone(PIN_SPEAKER, 800, 300);
  digitalWrite(PIN_LED, HIGH);
  delay(300);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  handleButton();

  readSensors();

  logData();

  if (systemArmed && currentHeight > heightTresshold) {
    executeTrigger();
  }

  delay(50); // small loop delay
}

// --- Functions ---

void handleButton() {
  if (digitalRead(PIN_BUTTON) == HIGH) { // pressed
    digitalWrite(PIN_LED, HIGH);
    systemArmed = true;
    Serial.println("SYSTEM ARMED");
  }
  else {
    digitalWrite(PIN_LED, LOW);
  }
  delay(500);
}

void readSensors() {
  currentHeight = altitude.getHeightAvg(20); // lighter average
  currentDistance = proximity.measureDistanceCm();
}

void logData() {
  Serial.print("Height: ");
  Serial.print(currentHeight);
  Serial.print(" m | Distance: ");
  Serial.print(currentDistance);
  Serial.println(" cm");
}

void executeTrigger() {
  Serial.println("TRIGGER ACTIVATED");

  // Servo trigger
  cameraShutter.write(70);
  delay(300);
  cameraShutter.write(90);

  triggerExecuted = true;
  systemArmed = false;
  digitalWrite(PIN_ONBOARD_LED, LOW);
}