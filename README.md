Code-1
const int irSensorPins[6] = {2, 3, 4, 5, 6, 7};  // IR sensor inputs (Pins 2-7 for Arduino Uno)
const int relayPins[6] = {8, 9, 10, 11, 12, 13};  // Relay outputs (Pins 8-13 for Arduino Uno)

bool firstDetection[6] = {true, true, true, true, true, true};  // Tracks whether it's the first or second detection per sensor
bool objectDetected[6] = {false, false, false, false, false, false}; // Tracks if an object is currently detected
int detectionCount[6] = {0, 0, 0, 0, 0, 0};  // Counter for detections per sensor

void setup() {
  Serial.begin(115200);  // Start Serial Monitor
  
  for (int i = 0; i < 6; i++) {
    pinMode(irSensorPins[i], INPUT);
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);  // Ensure relays are OFF initially (Active LOW)
  }
  Serial.println("System Ready - All Relays OFF");
}

void loop() {
  for (int i = 0; i < 6; i++) {
    int sensorState = digitalRead(irSensorPins[i]);

    if (sensorState == HIGH) { // Object detected
      if (!objectDetected[i]) {  // If this is a new detection
        objectDetected[i] = true;
        detectionCount[i]++;
        Serial.print("Detection count for sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(detectionCount[i]);

        int onTime = (i < 3) ? 2000 : 4000; // Relays 1-3: 2s, Relays 4-6: 4s

        Serial.print("Object detected at sensor ");
        Serial.print(i + 1);
        Serial.print("! Relay ON for ");
        Serial.print(onTime / 1000);
        Serial.println(" seconds.");
        digitalWrite(relayPins[i], LOW);  // Turn relay ON (Active LOW)
        delay(onTime);  // Wait for the respective time
        digitalWrite(relayPins[i], HIGH); // Turn relay OFF
        Serial.println("Relay OFF.");

        if (detectionCount[i] % 5 == 0) { // After every 5 detections
          int extendedOnTime = (i < 3) ? 5000 : 10000; // Relays 1-3: 5s, Relays 4-6: 10s
          Serial.print("5 detections reached for sensor ");
          Serial.print(i + 1);
          Serial.print("! Relay ON for ");
          Serial.print(extendedOnTime / 1000);
          Serial.println(" seconds.");
          digitalWrite(relayPins[i], LOW); // Turn relay ON
          delay(extendedOnTime); // Keep it ON for the respective time
          digitalWrite(relayPins[i], HIGH); // Turn relay OFF
          Serial.println("Relay OFF.");
        }
      }
    } 
    else { // No object detected
      if (objectDetected[i]) { // If object was detected earlier
        objectDetected[i] = false;
        int offTime = (i < 3) ? 4000 : 8000; // Relays 1-3: 4s, Relays 4-6: 8s
        Serial.print("No object detected at sensor ");
        Serial.print(i + 1);
        Serial.print(". Relay ON for ");
        Serial.print(offTime / 1000);
        Serial.println(" seconds.");

        digitalWrite(relayPins[i], LOW); // Turn relay ON (Active LOW)
        delay(offTime); // Keep it ON for the respective time
        digitalWrite(relayPins[i], HIGH); // Turn relay OFF
        Serial.println("Relay OFF.");
      }
    }
  }


  
Code -2
#include <Servo.h>

Servo doorServos[3];

// Door system
const int irSensorOutside[3] = {3, 5, 7};
const int irSensorInside[3]  = {4, 6, 8};
const int servoPins[3]       = {9, 10, 11};
bool doorLocked[3] = {false, false, false};

// Sanitizer system (using D12 & D13)
const int sanitizerIR     = 12;
const int sanitizerRelay  = 13;
bool sanitizerDetected    = false;

void setup() {
  Serial.begin(115200);

  // Door setup
  for (int i = 0; i < 3; i++) {
    doorServos[i].attach(servoPins[i]);
    doorServos[i].write((i == 0) ? 0 : 100); // Reversed direction for door 0
    pinMode(irSensorOutside[i], INPUT_PULLUP);
    pinMode(irSensorInside[i], INPUT_PULLUP);
  }

  // Sanitizer setup
  pinMode(sanitizerIR, INPUT_PULLUP);       // Use pull-up to avoid floating
  pinMode(sanitizerRelay, OUTPUT);
  digitalWrite(sanitizerRelay, HIGH);       // Relay OFF initially (active LOW)

  Serial.println("System Ready: 3 Doors (100°-0°) + 1 Sanitizer");
}

void loop() {
  // --- Door Logic ---
  for (int i = 0; i < 3; i++) {
    bool outsideDetected = (digitalRead(irSensorOutside[i]) == LOW);
    bool insideDetected  = (digitalRead(irSensorInside[i]) == LOW);

    if (outsideDetected && !doorLocked[i]) {
      Serial.print("Person outside Door "); Serial.println(i + 1);
      openDoor(i); delay(2000); closeDoor(i);
      doorLocked[i] = true;
    }

    if (insideDetected && doorLocked[i]) {
      Serial.print("Person inside Door "); Serial.println(i + 1);
      openDoor(i); delay(2000); closeDoor(i);
      doorLocked[i] = false;
    }
  }

  // --- Sanitizer Logic ---
  bool handDetected = (digitalRead(sanitizerIR) == LOW);

  if (handDetected && !sanitizerDetected) {
    sanitizerDetected = true;
    Serial.println("Hand detected! Dispensing sanitizer...");
    digitalWrite(sanitizerRelay, LOW);   // Relay ON
    delay(2000);                         // Dispense time
    digitalWrite(sanitizerRelay, HIGH);  // Relay OFF
    Serial.println("Sanitizer dispensed.");
  } 
  else if (!handDetected && sanitizerDetected) {
    // Reset state only when hand removed
    sanitizerDetected = false;
    Serial.println("Hand removed from sanitizer area.");
  }

  delay(50); // Small loop delay
}

// Servo control functions
void openDoor(int i) {
  if (i == 0) {
    for (int pos = 0; pos <= 100; pos++) {
      doorServos[i].write(pos);
      delay(10);
    }
  } else {
    for (int pos = 100; pos >= 0; pos--) {
      doorServos[i].write(pos);
      delay(10);
    }
  }
}

void closeDoor(int i) {
  if (i == 0) {
    for (int pos = 100; pos >= 0; pos--) {
      doorServos[i].write(pos);
      delay(10);
    }
  } else {
    for (int pos = 0; pos <= 100; pos++) {
      doorServos[i].write(pos);
      delay(10);
    }
  }
}


Code -3
#include <Servo.h>

Servo doorServo;

// ==== Pin Definitions ====
const int irOutside = 9;
const int irInside = 8;
const int relay10Person = 4;
const int relay30Sec = 5;
const int servoPin = 10;

const int sensorLow = 11;
const int sensorHigh = 12;
const int pumpRelay = 13;

const int sanitizerIRSensors[2] = {6, 7};
const int sanitizerRelays[2] = {2, 3};

// ==== Constants ====
const int openAngle = 0;
const int closeAngle = 100;
const int doorHoldTime = 2000;
const int servoDelay = 15;

const unsigned long interval30Sec = 30000;
const unsigned long durationRelay30Sec = 5000;
const unsigned long dispenseDuration = 2000;       // ← Updated to 2 seconds
const unsigned long relay10Duration = 5000;        // 5 seconds ON time

// ==== State Variables ====
int personCount = 0;
bool pumpState = false;
bool sanitizerActive[2] = {false, false};
unsigned long sanitizerStartTime[2] = {0, 0};

unsigned long last30SecTime = 0;
unsigned long relay30StartTime = 0;
bool relay30Active = false;

bool relay10Active = false;
unsigned long relay10StartTime = 0;

// ==== Door State ====
enum DoorState { IDLE, OPENING, HOLDING, CLOSING };
DoorState doorState = IDLE;

int currentServoPos = closeAngle;
unsigned long doorLastMoveTime = 0;
unsigned long doorHoldStart = 0;

void setup() {
  Serial.begin(115200);
  doorServo.attach(servoPin);
  doorServo.write(closeAngle);

  pinMode(irOutside, INPUT);
  pinMode(irInside, INPUT);
  pinMode(relay10Person, OUTPUT);
  pinMode(relay30Sec, OUTPUT);

  pinMode(sensorLow, INPUT_PULLUP);
  pinMode(sensorHigh, INPUT_PULLUP);
  pinMode(pumpRelay, OUTPUT);

  for (int i = 0; i < 2; i++) {
    pinMode(sanitizerIRSensors[i], INPUT);
    pinMode(sanitizerRelays[i], OUTPUT);
    digitalWrite(sanitizerRelays[i], HIGH); // Relay OFF (Active LOW)
  }

  digitalWrite(relay10Person, HIGH);
  digitalWrite(relay30Sec, HIGH);
  digitalWrite(pumpRelay, HIGH);

  Serial.println("System Initialized");
}

void loop() {
  unsigned long currentMillis = millis();

  // ==== 1. HAND SANITIZER LOGIC (Non-blocking) ====
  for (int i = 0; i < 2; i++) {
    bool detected = (digitalRead(sanitizerIRSensors[i]) == LOW);

    if (detected && !sanitizerActive[i]) {
      sanitizerActive[i] = true;
      sanitizerStartTime[i] = currentMillis;
      digitalWrite(sanitizerRelays[i], LOW); // Relay ON
      Serial.print("Sanitizer ");
      Serial.print(i + 1);
      Serial.println(": Hand detected → Dispensing started...");
    }

    if (sanitizerActive[i] && (currentMillis - sanitizerStartTime[i] >= dispenseDuration)) {
      digitalWrite(sanitizerRelays[i], HIGH); // Relay OFF
      sanitizerActive[i] = false;
      Serial.print("Sanitizer ");
      Serial.print(i + 1);
      Serial.println(": Dispensing complete.");
    }
  }

  // ==== 2. NON-BLOCKING DOOR SYSTEM ====
  bool outsideDetected = (digitalRead(irOutside) == LOW);
  bool insideDetected = (digitalRead(irInside) == LOW);

  switch (doorState) {
    case IDLE:
      if (outsideDetected || insideDetected) {
        Serial.println("Person detected → Opening door...");
        personCount++;
        doorState = OPENING;
        doorLastMoveTime = currentMillis;
      }
      break;

    case OPENING:
      if (currentMillis - doorLastMoveTime >= servoDelay) {
        if (currentServoPos > openAngle) {
          currentServoPos--;
          doorServo.write(currentServoPos);
          doorLastMoveTime = currentMillis;
        } else {
          doorHoldStart = currentMillis;
          doorState = HOLDING;
          Serial.println("Door fully open → Holding...");
        }
      }
      break;

    case HOLDING:
      if (currentMillis - doorHoldStart >= doorHoldTime) {
        doorState = CLOSING;
        doorLastMoveTime = currentMillis;
        Serial.println("Closing door...");
      }
      break;

    case CLOSING:
      if (currentMillis - doorLastMoveTime >= servoDelay) {
        if (currentServoPos < closeAngle) {
          currentServoPos++;
          doorServo.write(currentServoPos);
          doorLastMoveTime = currentMillis;
        } else {
          doorState = IDLE;
          Serial.println("Door closed.");
        }
      }
      break;
  }

  // ==== 3. RELAY FOR EVERY 10 PERSONS ====
  if (!relay10Active && personCount >= 10) {
    Serial.println("10 persons detected → Relay ON");
    digitalWrite(relay10Person, LOW);    // Relay ON
    relay10Active = true;
    relay10StartTime = currentMillis;
    personCount = 0;
  }

  if (relay10Active && (currentMillis - relay10StartTime >= relay10Duration)) {
    digitalWrite(relay10Person, HIGH);   // Relay OFF
    relay10Active = false;
    Serial.println("Relay OFF after 5 seconds");
  }

  // ==== 4. 30-SECOND TIMER RELAY ====
  if (!relay30Active && currentMillis - last30SecTime >= interval30Sec) {
    digitalWrite(relay30Sec, LOW);
    relay30StartTime = currentMillis;
    relay30Active = true;
    Serial.println("30-second timer relay ON");
  }

  if (relay30Active && currentMillis - relay30StartTime >= durationRelay30Sec) {
    digitalWrite(relay30Sec, HIGH);
    relay30Active = false;
    last30SecTime = currentMillis;
    Serial.println("30-second timer relay OFF");
  }

  // ==== 5. WATER LEVEL CONTROL ====
  bool lowLevel = digitalRead(sensorLow) == HIGH;
  bool highLevel = digitalRead(sensorHigh) == LOW;

  if (lowLevel) {
    pumpState = true;
  }

  if (highLevel) {
    pumpState = false;
  }

  digitalWrite(pumpRelay, pumpState ? LOW : HIGH);

  delay(20); // Optional small delay
}

  delay(50); // Small delay to avoid slow switching of relays 4, 5, 6
}

