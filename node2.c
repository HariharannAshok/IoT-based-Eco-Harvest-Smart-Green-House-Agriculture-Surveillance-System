#include <WiFi.h>
#include <esp_now.h>
/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// Example for demonstrating the TSL2591 library - public domain!
//Define I2C interfaces
/*Xiao ESP32S3*/
#define I2C_SDA 5  //Define I2C SDA interface in ESP32 board
#define I2C_SCL 6  //Define I2C SCL interface in ESP32 board
#define Aout_Gas A0

#define TempPin3 7  //GPIO pin, check the pin number an dfunction of your board
#define LightPin3 8

#define TempPin4 3  //GPIO pin, check the pin number an dfunction of your board
#define LightPin4 4

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);  // pass in a number for the sensor identifier (for your use later)

/*
    Configures the gain and integration time for the TSL2591
*/
void configureSensor(void) {
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);  // 25x gain
  // Changing the integration time gives you a longer time over which to sense light
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
}
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
float unifiedSensorAPIRead(void) {
  /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);
  /* Display the results (light is measured in lux) */
  if ((event.light == 0) | (event.light > 4294966000.0) | (event.light < -4294966000.0)) {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    Serial.println(F("Invalid data (adjust gain or timing)"));
    return 0;
  } else {
    Serial.print("TSL2591 light density: ");
    Serial.print(event.light);
    Serial.println(F(" lux"));
    return event.light;
  }
}


// Replace with the MAC address of the receiver ESP32S3 board
uint8_t broadcastAddress[] = { 0x34, 0x85, 0x18, 0x91, 0xC6, 0xE0 };  //34:85:18:8E:29:18 60B: 34:85:18:AC:BD:6C; 59A: 0x34, 0x85, 0x18, 0x91, 0x30, 0xA4

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  //char a[32];
  int id;
  float b; // tsl
  float c; // gas
  float d;
  float e;// h
  float f;
  float g;// b
  float h;
} struct_message;

/*------------------ CRC-8 Calculation-----------------------*/
// CRC-8 lookup table for polynomial 0x8C (reverse of 0x31)
const uint8_t crc8_table[256] = {
  0x00, 0x8C, 0x94, 0x18, 0xA4, 0x28, 0x30, 0xBC, 0xC4, 0x48, 0x50, 0xDC, 0x60, 0xEC, 0xF4, 0x78,
  0x04, 0x88, 0x90, 0x1C, 0xA0, 0x2C, 0x34, 0xB8, 0xC0, 0x4C, 0x54, 0xD8, 0x64, 0xE8, 0xF0, 0x7C,
  0x08, 0x84, 0x9C, 0x10, 0xAC, 0x20, 0x38, 0xB4, 0xCC, 0x40, 0x58, 0xD4, 0x68, 0xE4, 0xFC, 0x70,
  0x0C, 0x80, 0x98, 0x14, 0xA8, 0x24, 0x3C, 0xB0, 0xC8, 0x44, 0x5C, 0xD0, 0x6C, 0xE0, 0xF8, 0x74,
  0x10, 0x9C, 0x84, 0x08, 0xB4, 0x38, 0x20, 0xAC, 0xD4, 0x58, 0x40, 0xCC, 0x70, 0xFC, 0xE4, 0x68,
  0x14, 0x98, 0x80, 0x0C, 0xB0, 0x3C, 0x24, 0xA8, 0xD0, 0x5C, 0x44, 0xC8, 0x74, 0xF8, 0xE0, 0x6C,
  0x18, 0x94, 0x8C, 0x00, 0xBC, 0x30, 0x28, 0xA4, 0xDC, 0x50, 0x48, 0xC4, 0x78, 0xF4, 0xEC, 0x60,
  0x1C, 0x90, 0x88, 0x04, 0xB8, 0x34, 0x2C, 0xA0, 0xD8, 0x54, 0x4C, 0xC0, 0x7C, 0xF0, 0xE8, 0x64,
  0x20, 0xAC, 0xB4, 0x38, 0x84, 0x08, 0x10, 0x9C, 0xE4, 0x68, 0x70, 0xFC, 0x40, 0xCC, 0xD4, 0x58,
  0x24, 0xA8, 0xB0, 0x3C, 0x80, 0x0C, 0x14, 0x98, 0xE0, 0x6C, 0x74, 0xF8, 0x44, 0xC8, 0xD0, 0x5C,
  0x28, 0xA4, 0xBC, 0x30, 0x8C, 0x00, 0x18, 0x94, 0xEC, 0x60, 0x78, 0xF4, 0x48, 0xC4, 0xDC, 0x50,
  0x2C, 0xA0, 0xB8, 0x34, 0x88, 0x04, 0x1C, 0x90, 0xE8, 0x64, 0x7C, 0xF0, 0x4C, 0xC0, 0xD8, 0x54,
  0x30, 0xBC, 0xA4, 0x28, 0x94, 0x18, 0x00, 0x8C, 0xF4, 0x78, 0x60, 0xEC, 0x50, 0xDC, 0xC4, 0x48,
  0x34, 0xB8, 0xA0, 0x2C, 0x90, 0x1C, 0x04, 0x88, 0xF0, 0x7C, 0x64, 0xE8, 0x54, 0xD8, 0xC0, 0x4C,
  0x38, 0xB4, 0xAC, 0x20, 0x9C, 0x10, 0x08, 0x84, 0xFC, 0x70, 0x68, 0xE4, 0x58, 0xD4, 0xCC, 0x40,
  0x3C, 0xB0, 0xA8, 0x24, 0x98, 0x14, 0x0C, 0x80, 0xF8, 0x74, 0x6C, 0xE0, 0x5C, 0xD0, 0xC8, 0x44
};
// Function to calculate CRC-8
uint8_t calculateCRC8(const void* data, size_t length) {
  uint8_t crc = 0;
  uint8_t* buffer = (uint8_t*)data;

  for (size_t i = 0; i < length; i++) {
    crc = crc8_table[crc ^ buffer[i]];
  }

  return crc;
}
// Create a struct_message called myData
struct_message myData; // sending data
struct_message incomingData; // receiving request

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("\rLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println(" ");
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&incomingData, incomingData, sizeof(incomingData));
  Serial.println("Request received");

  float gas_value = analogRead(Aout_Gas);
  int luxValue = unifiedSensorAPIRead();
  int sensorValue_tm3 = analogRead(TempPin3);
  int sensorValue_op3 = analogRead(LightPin3);
  float voltage_tm3 = sensorValue_tm3 * (3.3 / 4096.0); 
  float voltage_op3 = sensorValue_op3 * (3.3 / 4096.0);
  float Lux3 = (-796.1*(voltage_op3)+1995.9);
  float Temp3 = (-5.1948*(voltage_tm3)+36.753);

  int sensorValue_tm4 = analogRead(TempPin4);
  int sensorValue_op4 = analogRead(LightPin4);
  float voltage_tm4 = sensorValue_tm4 * (3.3 / 4096.0); 
  float voltage_op4 = sensorValue_op4 * (3.3 / 4096.0);
  float Lux4 = (-825.59*(voltage_op4)+2364.4);
  float Temp4 = (-2.5*(voltage_tm4)+31.25);

  myData.id = 2;
  myData.b = unifiedSensorAPIRead();
  myData.c = gas_value;
  myData.d = 0;
  myData.e = Lux3;
  myData.f = Temp3;
  myData.g = Lux4;
  myData.h = Temp4;

  uint8_t CRC = calculateCRC8(&myData, sizeof(myData));
  // Send data including CRC using ESP-NOW
  uint8_t dataToSend[sizeof(myData) + 1];  //Covert to binary and send
  memcpy(dataToSend, &myData, sizeof(myData));
  dataToSend[sizeof(myData)] = CRC;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, dataToSend, sizeof(dataToSend));

  if (result == ESP_OK) {
    Serial.println(" Sent with success");
    Serial.print("Calculated CRC: ");
    Serial.println(dataToSend[sizeof(myData)]);
    Serial.print("Gas Value: ");
    Serial.println(gas_value);
    Serial.print("Lux : ");
    Serial.println(luxValue);
    Serial.print(F("Lux3 : "));
    Serial.println(Lux3);
    Serial.print(F("  Temp3 "));
    Serial.println(Temp3);
    Serial.print(F("Lux4 : "));
    Serial.println(Lux4);
    Serial.print(F("  Temp4 "));
    Serial.println(Temp4);
  } else {
    Serial.println("Error sending the data");
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);  //I2C pin initialisation
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
}
