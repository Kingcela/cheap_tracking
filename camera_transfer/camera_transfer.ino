#include "esp32cam.h"
#include "esp32cam/tinyml/edgeimpulse/FOMO.h"
#include <HardwareSerial.h>
using namespace Eloquent::Esp32cam;
Cam cam;

void setup() {
  Serial.begin(115200);
  // establish hardware connection between camera and ESP32
  SerialPort.beign(115200);
  delay(3000);
  // Connect to Camera module
  Serial.println("Init Camera module");
  cam.aithinker();
  cam.highQuality();
  cam.highestSaturation();
  cam.vga();

  while (!cam.begin())
      Serial.println(cam.getErrorMessage());

}

void loop() {
    // transfering information into ESP32
    SerialPort.write(bbox.label);
    SerialPort.write(bbox.x);
    SerialPort.write(bbox.y);
    SerialPort.write(bbox.width);
    SerialPort.write(bbox.height);

}

