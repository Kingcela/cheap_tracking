// ESP32 #2: TCP SERVER + AN LED
#include <WiFi.h>
#include <camera_tracking_inferencing.h>
#include <HardwareSerial.h>     // for serial connetion with ESP 32 board
#include <cmath>

#define SERVER_PORT 4080

const char* ssid = "OneGhost";
const char* password = "20001001";

WiFiServer TCPserver(SERVER_PORT);

void setup() {
  Serial.begin(115200);
  
  SerialPort.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Print your local IP address:
  Serial.print("ESP32 #2: TCP Server IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("ESP32 #2: -> Please update the serverAddress in ESP32 #1 code");

  // Start listening for a TCP client (from ESP32 #1)
  TCPserver.begin();
}

void loop() {
  // Wait for a TCP client from ESP32 #1:
  WiFiClient client = TCPserver.available();
  
  if (client) {
    // check for the object from camera
    Serial.println("Client connected");
    String camera_label;
    float camera_x = SerialPort.read();
    float camera_y = 0.0;
    float camera_width = 0.0;
    float camera_height = 0.0;
    float dt = 0.0;
    float update_x = 0.0;
    float update_y = 0.0;
    float velo_x = 0.0;
    float velo_y = 0.0; 
    String message = client.readStringUntil('\n'); // Read the message until end character
    int firstCommaIndex = message.indexOf(',');
    int secondCommaIndex = message.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = message.indexOf(',', secondCommaIndex + 1);
    if (fomo.hasObjects()) {
      // print information from camera first
      fomo.forEach([&client, &camera_label, &camera_x, &camera_y, &camera_width, &camera_height, &dt, &update_x, &update_y, &velo_x, &velo_y](size_t ix, ei_impulse_result_bounding_box_t bbox) {
        camera_label = SerialPort.read(bbox.label);
        camera_x = SerialPort.read(bbox.x);
        camera_y = SerialPort.read(bbox.y);
        camera_width = SerialPort.read(bbox.width);
        camera_height = SerialPort.read(bbox.height);
        Serial.print(" > info from Camera: ");
        Serial.print(camera_label);
        Serial.print(" at (");
        Serial.print(camera_x);
        Serial.print(", ");
        Serial.print(camera_y);
        Serial.print("), size ");
        Serial.print(camera_width);
        Serial.print(" x ");
        Serial.print(camera_height);
        Serial.println();
        // data from accelerometer
        while (client.connected()) {
          if (client.available()) {
            
            String message = client.readStringUntil('\n'); // Read the message until end character
            int firstCommaIndex = message.indexOf(',');
            int secondCommaIndex = message.indexOf(',', firstCommaIndex + 1);
            String accxStr = message.substring(0, firstCommaIndex);
            String accyStr = message.substring(firstCommaIndex + 1, secondCommaIndex);
            String dtimeStr = message.substring(secondCommaIndex + 1);
            // Convert to float or integer
            float accx = accxStr.toFloat();
            float accy = accyStr.toFloat();
            long dtime = dtimeStr.toInt();
            // skip over the first loop when dt = 0
            if (dt == 0.0) {
              velo_x = accx * 0.0003848 * dtime;    // convert it into unit of inches/ms^2
              velo_y = accy * 0.0003848 * dtime;
              update_x = camera_x;
              update_y = camera_y;
              dt = dtime;
              continue;
            }
            float acc_dt = dtime - dt;
            dt = dtime;
            // calculate the offset from accelerometer data
            float offset_x = 0.5 * accx * 0.0003848 * acc_dt * acc_dt;  // convert it into unit of inches/ms^2
            float offset_y = 0.5 * accy * 0.0003848 * acc_dt * acc_dt;  // convert it into unit of inches/ms^2
            velo_x = velo_x + offset_x;
            velo_y = velo_y + offset_y;

            float disp_x = pow((velo_x - camera_x), 2);
            float disp_y = pow((velo_y - camera_y), 2);
            float disp = sqrt(disp_x + disp_y);
            Serial.print(" < info from Accelerometer: ");
            Serial.print(camera_width);
            Serial.print(" x ");
            Serial.print(camera_height);
            Serial.print(", ");
            Serial.print(camera_label);
            Serial.print(" moved");
            Serial.print(velo_x);
            Serial.print(" inches");
            Serial.print(" in x direction and ");
            Serial.print(velo_y);
            Serial.print(" inches in x direction ");
            Serial.println();
          }
        }
      });
    }
    else {
        Serial.println("No objects detected");
    }
  } 
  // when not receiving info from accelerometer
  else {
    if (!cam.capture()) {
        Serial.println(cam.getErrorMessage());
        delay(1000);
        return;
    }
    // run FOMO model
    if (!fomo.detectObjects(cam)) {
        Serial.println(fomo.getErrorMessage());
        delay(1000);
        return;
    }
    // print found bounding boxes
    if (fomo.hasObjects()) {
        Serial.printf("Found %d objects in %d millis\n", fomo.count(), fomo.getExecutionTimeInMillis());

        fomo.forEach([](size_t ix, ei_impulse_result_bounding_box_t bbox) {
          Serial.print(" > info from Camera: ");
          Serial.print(camera_label);
          Serial.print(" at (");
          Serial.print(camera_x);
          Serial.print(", ");
          Serial.print(camera_y);
          Serial.print("), size ");
          Serial.print(camera_width);
          Serial.print(" x ");
          Serial.print(camera_height);
          Serial.println();
        });
    }
    else {
        Serial.println("No objects detected");
    }
  }
}

