#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include <NoDelay.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include "WiFi.h"
#include <esp_now.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define LIGHT_PIN  6 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 12 // Popular NeoPixel ring size

#define M_PI 3.14159265358979323846

uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0xAA, 0xE4, 0x98};

int signalState = -1;  // -1 - OFF, 0 - LEFT, 1 - RIGHT

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu;
Adafruit_Sensor *temperature, *accelerometer, *gyroscope;
Adafruit_NeoPixel pixels(NUMPIXELS, LIGHT_PIN, NEO_GRB + NEO_KHZ800);

typedef struct struct_message_in {
    int signal;
} struct_message_in;

typedef struct struct_message_out {
    int signal;
    int ledIndex;
} struct_message_out;

struct_message_out messageOut;
struct_message_in messageIn;
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status !=0){
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&messageIn, incomingData, sizeof(messageIn));
  Serial.print("Message received: ");
  Serial.println(messageIn.signal);
  buttonEventHandler();
}

/*
// AHRS
Adafruit_Madgwick filter;
Adafruit_Sensor_Calibration_EEPROM cal;
*/
float roll = 0.0;

// void ICACHE_RAM_ATTR buttonStateChanges();

bool powerSaving = false;
bool braking = true;

volatile bool buttonEventFlag = false;         // Flag when button has been pushed
volatile bool waitForButtonCycle = false;      // Track cycle of button states (push then release)
volatile unsigned long buttonPushTime = 0;     // Time the button was pushed
volatile unsigned long buttonReleaseTime = 0;  // Time the button was released
volatile unsigned long buttonEventTime = 0;    // Time the event was tracked by the handler

void displaySensorDetails(void) {
  accelerometer = mpu.getAccelerometerSensor();
  gyroscope = mpu.getGyroSensor();
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  delay(500);
}

void buttonStateChanges() {
  unsigned long currentTime = millis();
  if (currentTime - buttonPushTime < 500) {
    return;
  }
  buttonEventFlag = true;
  buttonPushTime = currentTime;
}

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  // while (!Serial) {
  //   delay(1);
  // }

  /* Initialise the sensor */
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
      ;
  }
  displaySensorDetails();

  pixels.begin();  // Initialize pins for output
  pixels.fill(0xff0000, 0, 0);
  pixels.setBrightness(70);
  pixels.show();  // Turn all LEDs off ASAP

  // AHRS
  //Wire.setClock(400000);  // 400KHz
  //filter.begin(100);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

}

noDelay LEDWait(250);
noDelay LEDBrakes(1000);
//noDelay BrakeCheckInterval(50);
noDelay TurnCheckInterval(40);
//noDelay FusionInterval(10);

int ledIndex = 0;
bool cleanState = false;
int signalTimeoutCounter = 0;
int signalTimeoutThres = 100;

void loop() {
  // put your main code here, to run repeatedly:
  //buttonEventHandler();
  //detectBrake();
  //fuse();

  if (signalTimeoutCounter > signalTimeoutThres) {
    clearSignal();
  }

  if (signalState != -1 && updateTurnStatus(signalState)) {
  }

  if (signalState == 0) {
    executeLeftSignal();
    cleanState = false;
  } else if (signalState == 1) {
    executeRightSignal();
    cleanState = false;
  } else {
    clearSignal();
    if (LEDBrakes.update()) {
      if(powerSaving){
        pixels.clear();
        pixels.setBrightness(70);
        pixels.show();
       //esp_sleep_enable_timer_wakeup(500 * 1000);
        //esp_light_sleep_start();
      }else if(braking) {
        pixels.fill(0xff0000, 0, 0);
      } else {
        pixels.fill(0x320000, 0, 0);
      }
      pixels.setBrightness(70);
      pixels.show();
    }
  }
}

// ---------- trackRotation Variables ----------

// --- Constants ---

const float initialRotation = 0.3;
const float initialDuration = 100;
const float turnEndThres = 0.07;     // Max Average Rotation Threshold for detecting an ended turn
const float turnEndDuration = 2500;  // Turn End Averaging Total Duration
const float minAngle = 60;

// --- Variables ---

bool turnStarted = false;
bool turnEnded = false;

// DT
float initialAverage = 0;
unsigned long initialSampleTime = 0;
int initialSampleCount = 0;

// ET
float cumulativeAngle = 0;
float turnEndAverage = 0;
unsigned long turnEndSampleTime = 0;
int turnEndSampleCount = 0;

unsigned long lastRecordTime = 0;
float lastRotation = 0;

void computeAverage(float value, float *average, unsigned long *lastSampleTime, int *sampleCount) {
  *average = (*average * (*sampleCount - 1) + value) / *sampleCount;
  *sampleCount++;
  return;
}

bool detectTurnEnd(float rotation, unsigned long timeNow) {
  if (turnEndSampleCount == 0) {
    turnEndSampleTime = timeNow;
    turnEndSampleCount = 2;
    turnEndAverage = rotation;
    return false;
  }
  computeAverage(rotation, &turnEndAverage, &turnEndSampleTime, &turnEndSampleCount);
  float updatedAvg = turnEndAverage;
  if (timeNow - turnEndSampleTime >= turnEndDuration) {
    turnEndAverage = 0;
    turnEndSampleTime = 0;
    turnEndSampleCount = 0;
    return abs(updatedAvg) <= turnEndThres;
  }
  return false;
}

bool detectTurn(float rotation, unsigned long timeNow, int direction) {
  if (initialSampleCount == 0) {
    if (direction == 1 && rotation < initialRotation) return false;
    if (direction == -1 && rotation > initialRotation * -1) return false;
    initialSampleTime = timeNow;
    initialSampleCount = 2;
    initialAverage = rotation;
    return false;
  }
  float updatedAvg = (initialAverage * (initialSampleCount - 1) + rotation) / initialSampleCount;

  if (timeNow - initialSampleTime >= initialDuration) {
    initialAverage = 0;
    initialSampleCount = 0;
    if (direction == -1) return updatedAvg <= initialRotation * -1;
    return updatedAvg >= initialRotation;
  }

  initialAverage = updatedAvg;
  initialSampleCount++;
}

void trackRotation(float rotation, unsigned long timeNow, int direction) {
  // Quit
  if (turnStarted && turnEnded) {
    clearSignal();
    return;
  }

  // Detect Turn
  if (!turnStarted) {
    turnStarted = detectTurn(rotation, timeNow, direction);
    if (turnStarted) Serial.println("Turn Started");
    lastRecordTime = timeNow;
    lastRotation = rotation;
    return;
  }

  cumulativeAngle += lastRotation * 57.2958 * (timeNow - lastRecordTime) / 1000;
  lastRecordTime = timeNow;
  lastRotation = rotation;

  Serial.println(cumulativeAngle);

  bool thresAngleExceed = !(cumulativeAngle < 0 ^ direction < 0) && abs(cumulativeAngle) >= minAngle;

  if (thresAngleExceed) {
    turnEnded = detectTurnEnd(rotation, timeNow);
    if (turnEnded) {
      Serial.println("Turn Ended Sia!");
    }
  } else {
    turnEndAverage = 0;
    turnEndSampleTime = 0;
    turnEndSampleCount = 0;
  }
  return;
}

void resetRotationTracking() {
  Serial.println("Rotation Tracking Reset!");
  turnStarted = false;
  turnEnded = false;

  // DT
  initialAverage = 0;
  initialSampleTime = 0;
  initialSampleCount = 0;

  // ET
  cumulativeAngle = 0;
  turnEndAverage = 0;
  turnEndSampleTime = 0;
  turnEndSampleCount = 0;

  lastRecordTime = 0;
  lastRotation = 0;
}

// ---------- trackRotation End ----------

void buttonEventHandler() {
  if(messageIn.signal == 2){
    powerSaving = !powerSaving;
    clearSignal();
    return;
  }

  signalState = messageIn.signal;
  clearSignal();
  buttonEventFlag = false;
}

bool updateTurnStatus(bool toTurnRight) {
  if (!TurnCheckInterval.update()) {
    return false;
  }
  sensors_event_t event;
  gyroscope->getEvent(&event);
  float gyroY = -event.gyro.y;

  int direction = 1;
  if (signalState == 1) direction = -1;

  trackRotation(gyroY, millis(), direction);
}

/*
float brakeAvg = 0;
unsigned long brakeSampleTime = 0;
int brakeSampleCount = 0;

bool avgBrake(float acc, unsigned long timeNow) {
  if (brakeSampleCount == 0) {
    brakeSampleTime = timeNow;
    brakeSampleCount = 2;
    brakeAvg = acc;
    return false;
  }
  float updatedAvg = (brakeAvg * (brakeSampleCount - 1) + acc) / brakeSampleCount;

  if (timeNow - brakeSampleTime >= 1000) {
    brakeSampleCount = 0;
    brakeSampleTime = 0;
    return true;
  }

  brakeAvg = updatedAvg;
  brakeSampleCount++;
}

void detectBrake() {
  if (!BrakeCheckInterval.update()) return;
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);
  Serial.print("Roll: ");
  Serial.println(roll);
  float ay = aevent.acceleration.y;
  float az = aevent.acceleration.z;
  Serial.print("Raw: ");
  Serial.print(ay, 4);
  Serial.print(", ");
  Serial.print(az, 4);
  Serial.print(", ");
  Serial.println();

  float ayLinear = 0.0;
  float azLinear = 0.0;

  float rotationMatrix[4] = { 0.0 };

  double angleInRads = roll * M_PI / 180;

  rotationMatrix[0] = cos(angleInRads);
  rotationMatrix[1] = -1 * sin(angleInRads);
  rotationMatrix[3] = sin(angleInRads);
  rotationMatrix[4] = cos(angleInRads);

  ayLinear = ay * rotationMatrix[0] + az * rotationMatrix[1];
  azLinear = ay * rotationMatrix[3] + az * rotationMatrix[4];


  Serial.print("Computed: ");
  Serial.print(ayLinear, 4);
  Serial.print(", ");
  Serial.print(azLinear, 4);
  Serial.print(", ");
  Serial.println();

  if (!braking && avgBrake(ayLinear, millis()) && brakeAvg > 0.8) {
    braking = true;
  }

  if (braking && avgBrake(ayLinear, millis()) && brakeAvg < 0.05) {
    Serial.println(brakeAvg);
    braking = false;
  }
}
*/

void clearSignal() {
  if (!cleanState) {
    pixels.clear();
    pixels.show();
    resetRotationTracking();
    cleanState = true;
    signalState = -1;
    ledIndex = 0;
    signalTimeoutCounter = 0;
    messageOut.signal = signalState;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &messageOut, sizeof(messageOut));
    if (result != ESP_OK) {
    Serial.println("Error sending the data");
    }
  }
}

void executeRightSignal() {
  if (!LEDWait.update()) {
    return;
  }

  if (ledIndex == 0) {
    pixels.setPixelColor(0, 0xff6400);
    for(int i = 6; i < 12; i++){
      pixels.setPixelColor(i, 0xff6400);  //  Set pixel's color (in RAM)
    }
    pixels.setBrightness(50);
    pixels.show();
    ledIndex++;
  } else {
    ledIndex = 0;
    pixels.fill(0, 0, 0);
    pixels.show();
    signalTimeoutCounter++;
  }
  messageOut.ledIndex = ledIndex;
  messageOut.signal = signalState;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &messageOut, sizeof(messageOut));
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
    }
}

void executeLeftSignal() {
  if (!LEDWait.update()) {
    return;
  }

  if (ledIndex == 0) {
    for(int i = 0; i < 7; i++){
      pixels.setPixelColor(i, 0xff6400);  //  Set pixel's color (in RAM)
    }
    pixels.setBrightness(50);
    pixels.show();
    ledIndex++;
  } else {
    ledIndex = 0;
    pixels.fill(0, 0, 0);
    pixels.show();
    signalTimeoutCounter++;
  }

  messageOut.ledIndex = ledIndex;
  messageOut.signal = signalState;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &messageOut, sizeof(messageOut));
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
    }
}
