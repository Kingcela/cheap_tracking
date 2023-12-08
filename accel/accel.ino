#include "SparkFunLSM6DSO.h"
#include "SPI.h"
#include <WiFi.h>

#define Data_out 13
#define Data_in 12
#define CLK 11
#define ChipSelect 10

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

const char* ssid = "OneGhost";     // CHANGE TO YOUR WIFI SSID
const char* password = "20001001"; // CHANGE TO YOUR WIFI PASSWORD
const char* serverAddress = "192.168.209.192"; // CHANGE TO ESP32#2'S IP ADDRESS
const int serverPort = 4080;

WiFiClient TCPclient;

void setup() {
  Serial.begin(115200);
  SPI.begin(CLK, Data_out, Data_in, ChipSelect); 
  pinMode(ChipSelect, OUTPUT); // Set CS pin as an output
  delay(500);   
  if( myIMU.beginSPI(ChipSelect, 100000, SPI))
    Serial.println("Ready.");
  else { 
    Serial.println("\nCould not connect to IMU.");
    Serial.println("Freezing");
    while(1);
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");


  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // connect to TCP server (Arduino #2)
  if (TCPclient.connect(serverAddress, serverPort)) {
    Serial.println("Connected to TCP server");
  } else {
    Serial.println("Failed to connect to TCP server");
  }
}


void loop()
{
  if (!TCPclient.connected()) {
    Serial.println("Connection is disconnected");
    TCPclient.stop();

    // reconnect to TCP server (Arduino #2)
    if (TCPclient.connect(serverAddress, serverPort)) {
      Serial.println("Reconnected to TCP server");
    } else {
      Serial.println("Failed to reconnect to TCP server");
    }
  }
  //Get all parameters
  digitalWrite(ChipSelect, LOW);
  char accx[16];
  char accy[16];
  char dtime[16];
  dtostrf(myIMU.readFloatAccelX(), 8, 2, accx);
  dtostrf(myIMU.readFloatAccelY(), 8, 2, accy);
  itoa(millis(), dtime, 10);
  char message[55]; 
  strcpy(message, accx);
  strcat(message, ", ");
  strcat(message, accy);
  strcat(message, ", ");
  strcat(message, dtime);
  
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 3);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 3);
  Serial.println(millis());

  TCPclient.write(message);
  TCPclient.flush();
  Serial.println("Sending the accelerometer data to client");
}
