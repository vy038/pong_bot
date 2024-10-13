/*
    ESP-NOW Broadcast Receiver
    By: Victor Yu
    Created: 2024-08-26
    Modified: 2024-08-26

    This sketch demonstrates how to receive broadcast messages from a master device using the ESP-NOW protocol.

    The master device will broadcast a message every 5 seconds to all devices within the network.

    The slave devices will receive the broadcasted messages. If they are not from a known master, they will be registered as a new master
    using a callback function.
*/

#include "ESP32_NOW.h"
#include "WiFi.h"

#include <Arduino.h>

#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#include <vector>

/* Definitions */

#define ESPNOW_WIFI_CHANNEL 6
char* info;
const int buttonA = 12; // good
const int buttonB = 33; // good
const int forward = 26; // good
const int backward = 27; // good
const int left = 25; // good
const int right = 32; // good 

/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    //Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    Serial.printf("  Message: %s\n", (char *)data);

    info = ((char *)data);

    Serial.println(((char *)data)[0]);

    if ((info[0]- '0') > 3) { // going forward
      analogWrite(forward, 255);
    } else if ((info[0] - '0') < 1) { // going backwards
      analogWrite(backward, 255);
    } else {
      analogWrite(forward, 0);
      analogWrite(backward, 0);
    }

    if ((info[1] - '0') > 3) { // going right
      analogWrite(right, 255);
    } else if ((info[1] - '0') < 1) { // going left
      analogWrite(left, 255);
    } else {
      analogWrite(right, 0);
      analogWrite(left, 0);
    }

    if ((info[2] - '0') == 0) { // button A being pressed
      digitalWrite(buttonA, HIGH);
    } else {
      digitalWrite(buttonA, LOW);
    }

    if ((info[3] - '0') == 0) { // button B being pressed
      digitalWrite(buttonB, HIGH);
    } else {
      digitalWrite(buttonB, LOW);
    }
  }
};

/* Global Variables */

// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

/* Main */

void setup() {
  Serial.begin(115200);

  pinMode(buttonA, OUTPUT);
  pinMode(buttonB, OUTPUT);
  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW - Receiver for pong bot");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, NULL);

  Serial.println("Setup complete. Waiting for a master to broadcast a message...");
}

void loop() {
  delay(1000);
}
