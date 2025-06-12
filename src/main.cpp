
// Including Libraries 
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <DHT.h>
#include <DHT_U.h>

#include <RTClib.h>

// Defining PINS
#define DHTPIN 12


// Defining Device Types
#define DHTTYPE    DHT11 

//Define Objects
DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
RTC_DS3231 rtc; //Create rtc object

// Set delay time structure
uint32_t delayMS;

char t[32];

void setup()
{
    // Initialize devices
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  dht.begin();

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set initial time

    while (!Serial); // Wait for serial port to be ready
  if (! rtc.begin()) {
    Serial.println("ERROR: Could not find RTC");
    while(1);
  }
  if (rtc.lostPower()) {  //If RTC Lost power, reset time from PC
    Serial.println("WARNING: RTC lost power, so we are assuming the current time is valid");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Set sensor structure
  sensor_t sensor;

 // Set delay between sensor readings
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // Delay
  delay(delayMS);

  // RTC ----------------------------------------------------------------------------------------------------------//
  // Read Date and Time - adjust over time
  DateTime now = rtc.now();
  sprintf(t, "%02d:%02d:%02d %02d/%02d/%02d",  now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year()); //adjust time over time 
  Serial.print(F("Date/Time: ")); // Print Time to Serial Monitor
  Serial.println(t);

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
  //----------------------------------------------------------------------------------------------------------//
}
