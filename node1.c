#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>

#define DHTPIN 5       //GPIO 5 is connected to the DHT sensor
#define DHTTYPE DHT22  //DHT 22(AM2302, AM2301)
#define Aout_Moisture A3

#define TempPin1 1  //GPIO pin, check the pin number an dfunction of your board
#define LightPin1 2

#define TempPin2 7  //GPIO pin, check the pin number an dfunction of your board
#define LightPin2 8

DHT dht(DHTPIN, DHTTYPE);

// Replace with the MAC address of the receiver ESP32S3 board
uint8_t broadcastAddress[] = { 0x34, 0x85, 0x18, 0x91, 0xC6, 0xE0 };  //34:85:18:8E:29:18 60B: 34:85:18:AC:BD:6C  59A: 0x34, 0x85, 0x18, 0x91, 0x30, 0xA4

typedef struct struct_message {
  
  int id;
  float b;  // humidity
  float c;  // temprature
  float d;  // Moisture
  float e;  // H
  float f;
  float g;  // R
  float h;
} struct_message;

typedef struct requestmessage {
  char* text;
} requestmessage;
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
struct_message myData; // sending message
requestmessage incomingData; // received message


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

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int moisture = analogRead(Aout_Moisture);

  int sensorValue_tm1 = analogRead(TempPin1);
  int sensorValue_op1 = analogRead(LightPin1);
  float voltage_tm1 = sensorValue_tm1 * (3.3 / 4096.0); 
  float voltage_op1 = sensorValue_op1 * (3.3 / 4096.0);
  float Lux1 = ((-344.63*voltage_op1)+746.8);
  float Temp1 = ((-59.173*voltage_tm1)+67.11);
  

  int sensorValue_tm2 = analogRead(TempPin2);
  int sensorValue_op2 = analogRead(LightPin2);
  float voltage_tm2 = sensorValue_tm2 * (3.3 / 4096.0); 
  float voltage_op2 = sensorValue_op2 * (3.3 / 4096.0);
  float Lux2 = ((219.89*voltage_op2)-13.504);
  float Temp2 = (64.574-(22.34*voltage_tm2));

  

  myData.id = 1;
  myData.b = h;
  myData.c = t;
  myData.d = moisture;
  myData.e = Lux1;
  myData.f = Temp1;
  myData.g = Lux2;
  myData.h = Temp2;

  //Calculate CRC
  uint8_t CRC = calculateCRC8(&myData, sizeof(myData));
  // Send data including CRC using ESP-NOW
  uint8_t dataToSend[sizeof(myData) + 1];
  memcpy(dataToSend, &myData, sizeof(myData));
  dataToSend[sizeof(myData)] = CRC;


  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, dataToSend, sizeof(dataToSend));

  if (result == ESP_OK) {
    Serial.println(" Sent with success");
    Serial.print("Calculated CRC: ");
    Serial.println(dataToSend[sizeof(myData)]);
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F("C "));
    Serial.print(F("Moisture :"));
    Serial.println(moisture);
    Serial.print(F("Lux1 : "));
    Serial.println(Lux1);
    Serial.print(F("  Temp1 "));
    Serial.println(Temp1);
    Serial.print(F("Lux2 : "));
    Serial.println(Lux2);
    Serial.print(F("  Temp2 "));
    Serial.println(Temp2);
  } 
  else 
  {
    Serial.println("Error sending the data");
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  dht.begin();

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
