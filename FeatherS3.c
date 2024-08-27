#include <HardwareSerial.h>
#include <WiFi.h>
#include <ThingSpeak.h>


const char* ssid = "Phone_1";         //Type wifi name
const char* password = "12345678";  //TYpe wifi password

HardwareSerial SerialPort(2);  //use UART2
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define BAUD_RATE 115200
int led = 13;

typedef struct UART_message {
  int id;
  float b;
  float c;
  float d;
  float e;
  float f;
  float g;
  float h;
  uint8_t CRC_checksum;  //CRC for error checking
} UART_message;

WiFiClient client;

unsigned long myChannelNumber = 1;               //The channel number in ThingSpeak (e.g.Channel 1)
const char* myWriteAPIKey = "MYAIFE1GXP95CVXD";  //Write API Key, can be obtained from ThingSpeak,differenct channel has different API keyTimer variables

unsigned long lastTime = 0;
unsigned long timerDelay = 20000;

void setup() {
  Serial.begin(BAUD_RATE);
  SerialPort.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      delay(5000);
    }
    Serial.println("\nWiFi Connected.");
  }
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  pinMode(led, OUTPUT);
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  delay(1000);
}

void loop() {

  // Variable definition for uploading data to ThingSpeak
  float humidity;
  float temperature;
  float moisture;
  float gasvalue;
  float LightDensity;
  float Lux1;
  float Temp1;
  float Lux2;
  float Temp2;
  float Lux3;
  float Temp3;
  float Lux4;
  float Temp4;

  // Serial Port Scanning
  if (SerialPort.available()) {
    // String message = SerialPort.readString();
    // Serial.println("Received message: " + message);
    size_t byteSize = sizeof(UART_message);
    byte* byteArray = new byte[byteSize];
    SerialPort.read(byteArray, byteSize);

    // Deserialize the byte array back into the structure
    UART_message UARTReceivedData;
    memcpy(&UARTReceivedData, byteArray, byteSize);

    /*--------Process different types of data----------*/
    switch (UARTReceivedData.id) {
      case 1:
        // Print the received structure data and check the data valid or not
        if (UARTReceivedData.b > 0 && (UARTReceivedData.c > -50 && UARTReceivedData.c < 50)) {
          Serial.print("Received from Board ID: ");
          Serial.println(UARTReceivedData.id);
          Serial.print("    Humidity: ");
          Serial.print(UARTReceivedData.b);  // b is linked to humidity
          Serial.println(" % ");
          Serial.print("    Temperature: ");  // c to temperature
          Serial.print(UARTReceivedData.c);
          Serial.println(" C ");
          Serial.print("    Moisture: ");  // d to Moisture
          Serial.println(UARTReceivedData.d);
          Serial.print("    Lux1: ");  // d to Moisture
          Serial.println(UARTReceivedData.e);
          Serial.print("    Temp1: ");  // d to Moisture
          Serial.println(UARTReceivedData.f);
          Serial.print("    Lux2: ");  // d to Moisture
          Serial.println(UARTReceivedData.g);
          Serial.print("    Temp2: ");  // d to Moisture
          Serial.println(UARTReceivedData.h);
          Serial.print("    CRC = ");  // CRC=1 means data transmission right
          Serial.println(UARTReceivedData.CRC_checksum);
          humidity = UARTReceivedData.b;
          temperature = UARTReceivedData.c;
          moisture = UARTReceivedData.d;
          Lux1 = UARTReceivedData.e;
          Temp1 = UARTReceivedData.f;
          Lux2 = UARTReceivedData.g;
          Temp2 = UARTReceivedData.h;
          ThingSpeak.setField(1, humidity);
          ThingSpeak.setField(2, temperature);
          ThingSpeak.setField(3, moisture);
        }
        break;
      
      default:
        if (UARTReceivedData.b > 0) {
          Serial.print("Received from Board ID: ");
          Serial.println(UARTReceivedData.id);
          Serial.print("    TSL2591 Light Density: ");
          Serial.print(UARTReceivedData.b);  // b is linked to light desnity
          Serial.println(" lux ");
          Serial.print("    Gas Value: ");
          Serial.println(UARTReceivedData.c);  // b is linked to light desnity
          Serial.print("    Lux3: ");  // d to Moisture
          Serial.println(UARTReceivedData.e);
          Serial.print("    Temp3: ");  // d to Moisture
          Serial.println(UARTReceivedData.f);
          Serial.print("    Lux4: ");  // d to Moisture
          Serial.println(UARTReceivedData.g);
          Serial.print("    Temp4: ");  // d to Moisture
          Serial.println(UARTReceivedData.h);
          Serial.print("    CRC = ");  // CRC=1 means data transmission right
          Serial.println(UARTReceivedData.CRC_checksum);
          LightDensity = UARTReceivedData.b;
          gasvalue = UARTReceivedData.c;
          Lux3 = UARTReceivedData.e;
          Temp3 = UARTReceivedData.f;
          Lux4 = UARTReceivedData.g;
          Temp4 = UARTReceivedData.h;
          ThingSpeak.setField(4, LightDensity);
          ThingSpeak.setField(5, gasvalue);
        }
        break;
    }
    digitalWrite(led, HIGH);
    delay(100);

    //ThingSpeak Data UIploading
    if ((millis() - lastTime) > timerDelay) {

      // Connect or reconnect to WiFi

      if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect");
        while (WiFi.status() != WL_CONNECTED) {
          WiFi.begin(ssid, password);
          delay(5000);
        }
        Serial.println("\nConnected.");
      }
      
      int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

      if (x == 200) {
        Serial.println("Channel update successful.");
      } else {
        Serial.println("Problem updating channel. HTTP error code " + String(x));
      }
      lastTime = millis();
    }
  }
  digitalWrite(led, LOW);  //digitalWrite(LED, HIGH);
}
