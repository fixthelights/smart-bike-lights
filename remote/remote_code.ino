#include "Adafruit_Debounce.h"
#include <NoDelay.h>
#include "WiFi.h"
#include <esp_now.h>

#define RIGHT_LED 3
#define LEFT_LED 2
#define MIDDLE_LED 1

#define LEFT_BUTTON 4
#define RIGHT_BUTTON 5

uint8_t broadcastAddress[] = { 0xEC, 0xDA, 0x3B, 0xAA, 0xDD, 0x20 };

// Create a debounce object for the button, which will be pressed when LOW
Adafruit_Debounce buttonLeft(LEFT_BUTTON, LOW);
Adafruit_Debounce buttonRight(RIGHT_BUTTON, LOW);

noDelay checkButtons(100);

typedef struct struct_message_out {
  int signal;
} struct_message_out;

typedef struct struct_message_in {
  int signal;
  int ledIndex;
} struct_message_in;

// Create a struct_message each for message to be sent and to be received
struct_message_out messageOut;
struct_message_in messageIn;
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&messageIn, incomingData, sizeof(messageIn));
  Serial.println("Message received: ");
  Serial.println(messageIn.signal);
  Serial.println(messageIn.ledIndex);
}

void setup() {
  // Serial.begin(9600);
  // while (!Serial) {
  //   delay(1);
  // }
  Serial.print("Hello");

  // put your setup code here, to run once:
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(LEFT_LED, OUTPUT);
  pinMode(MIDDLE_LED, OUTPUT);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

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
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);


  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (messageIn.signal != -1) {
    digitalWrite(MIDDLE_LED, LOW);
    if (messageIn.signal == 0) {
      digitalWrite(LEFT_LED, (bool)messageIn.ledIndex);
    } else {
      digitalWrite(RIGHT_LED, (bool)messageIn.ledIndex);
    }
  } else if (messageIn.signal == -1) {
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);
    digitalWrite(MIDDLE_LED, HIGH);
  }


  if (checkButtons.update()) {
    // Update the button state, this will do the digitalRead() for us
    // must be called ~every 10ms to keep state and catch presses
    buttonLeft.update();
    buttonRight.update();

    if (buttonLeft.justPressed() && buttonRight.justPressed()) {
      messageOut.signal = 2;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&messageOut, sizeof(messageOut));
      Serial.print("Sending: ");
      Serial.println(messageOut.signal);
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      } else {
        Serial.println("Error sending the data");
      }
    } else if (buttonLeft.justPressed()) {
      Serial.print("Hello! BUTTON WORKS SIA");
      // Send message via ESP-NOW
      messageOut.signal = 0;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&messageOut, sizeof(messageOut));
      Serial.print("Sending: ");
      Serial.println(messageOut.signal);
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      } else {
        Serial.println("Error sending the data");
      }
    } else if (buttonRight.justPressed()) {
      messageOut.signal = 1;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&messageOut, sizeof(messageOut));
      Serial.print("Sending: ");
      Serial.println(messageOut.signal);
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      } else {
        Serial.println("Error sending the data");
      }
    }
  }
}

