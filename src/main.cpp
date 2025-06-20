
// Including Libraries 
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#include <DHT.h>  // DHT11 Module
#include <DHT_U.h>      
#include <TinyGPSPlus.h>  // GPS Module
#include <SoftwareSerial.h> // Virtual Serial for GPS


// Defining PINS------------------------------------------------------------------------//
#define DHTPIN 12

// Grove GPS air 350 Module
static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;


// Defining Device Types ----------------------------------------------------------------//
#define DHTTYPE    DHT11 

//Define Objects ------------------------------------------------------------------------//
DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
TinyGPSPlus gps; // The TinyGPSPlus object

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


// Set delay time
uint32_t delayMS;

void displayInfo(); // Function prototype

void setup() {
    // Initialize devices
  Serial.begin(9600); // Physical Serial

  Serial.println("Serial Started");

  ss.begin(GPSBaud); // Virtual Serial
  
  Serial.println("Virtual Serial Started");

  Wire.begin();
  dht.begin();

  // Set DHT11 sensor -----------------------------------------------------------------------------------------------------//
  sensor_t sensor;

 // Set delay between sensor readings
  delayMS = 1000;
}

void loop() {
  // DHT11 ----------------------------------------------------------------------------------------------------------//
  // Get temperature event and print its value
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  // GPS ----------------------------------------------------------------------------------------------------------//
  while (ss.available() > 0)
    if (gps.encode(ss.read()))  {
      Serial.println("Displaying GPS Data...");
      displayInfo();
    }
      

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  // Delay
  delay(delayMS);
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