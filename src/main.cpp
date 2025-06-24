
// Including Libraries 
#include <Arduino.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>


//#include <DHT.h>  // DHT11 Module
//#include <DHT_U.h>      
#include <TinyGPSPlus.h>  // GPS Module
#include <SoftwareSerial.h> // Virtual Serial for GPS


// Defining PINS------------------------------------------------------------------------//
//#define DHTPIN 12

// Grove GPS air 350 Module
static const int RXPin = 4, TXPin = 8;
static const uint32_t GPSBaud = 9600;
const long interval = 10000;
unsigned long previousMillis = 0;


// Defining Device Types ----------------------------------------------------------------//
#define DHTTYPE    DHT11 

//Define Objects ------------------------------------------------------------------------//
//DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
TinyGPSPlus gps; // The TinyGPSPlus object

// The serial connection to the GPS device
SoftwareSerial GPSserial(RXPin, TXPin);


void setup() {
  // Initialize devices
  Serial.begin(9600); // Physical Serial
  GPSserial.begin(GPSBaud); // Virtual Serial
  //Wire.begin();
  //dht.begin();

  // Set DHT11 sensor -----------------------------------------------------------------------------------------------------//
  //sensor_t sensor;

  // Set delay between sensor readings
}

void loop() {
  // DHT11 ----------------------------------------------------------------------------------------------------------//
  // // Get temperature event and print its value
  // sensors_event_t event;

  // dht.temperature().getEvent(&event);
  // if (isnan(event.temperature)) {
  //   Serial.println(F("Error reading temperature!"));
  // }
  // else {
  //   Serial.print(F("Temperature: "));
  //   Serial.print(event.temperature);
  //   Serial.println(F("°C"));
  // }
  // // Get humidity event and print its value.
  // dht.humidity().getEvent(&event);
  // if (isnan(event.relative_humidity)) {
  //   Serial.println(F("Error reading humidity!"));
  // }
  // else {
  //   Serial.print(F("Humidity: "));
  //   Serial.print(event.relative_humidity);
  //   Serial.println(F("%"));
  // }
  // GPS ----------------------------------------------------------------------------------------------------------//
  // Feed the GPS parser as often as possible
  while (GPSserial.available()) {
    gps.encode(GPSserial.read());
  }

  // Print GPS info at fixed intervals 10sec
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (gps.location.isValid()) {
      String GPSData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
      Serial.println(GPSData);
      Serial.println("GPS Data Printed");
    } 
    else {
      Serial.println("Waiting for valid GPS data...");
    }
  }
}

// Custom Functions -----------------------------------------------------------------------------------------//
void displayInfo() {
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

// This custom version of delay() ensures that the gps object is being "fed"
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do {
    while (GPSserial.available()) {
      gps.encode(GPSserial.read());
    }
  } while (millis() - start < ms);
}
