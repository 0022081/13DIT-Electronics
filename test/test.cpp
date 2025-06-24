#include <SoftwareSerial.h>

SoftwareSerial GPSserial(4, 8); // RX, TX

void setup() {
  Serial.begin(9600);      // USB serial to computer
  GPSserial.begin(9600);   // GPS serial
  Serial.println("GPS Raw Monitor Starting...");
}

void loop() {
  while (GPSserial.available()) {
    char c = GPSserial.read();
    Serial.write(c);  // Write raw GPS output to Serial Monitor
  }
}
