// Including libraries
#include <header.h>

// Constants ---------------------------------------------------------------------------//
#define DHTPIN 12  // DHT11 Pin

// Grove GPS Constants
static const int GPSRXPin = 2, GPSTXPin = 4;  // Serial Port
static const uint32_t GPSBaud = 9600; // GPS Baud Rate
const long interval = 10000;  
unsigned long previousMillis = 0;

// Thermistor Constants ------------------------------------------------------------------//
const int thermistorPin = A0; // Thermistor pins
const float seriesResistor = 10000.0;  // 10k Ohm series resistor
const float nominalResistance = 10000.0; // Resistance of thermistor at 25ºC
const float nominalTemperature = 25.0;   // Nominal temperature (ºC)
const float betaCoefficient = 3892.0;    // Beta coefficient of the thermistor
const float adcMax = 4095.0;             // Max value from analogRead

// Soil Data Constants ------------------------------------------------------------------//
const uint8_t soilPin = A1;        // analog pin for envelope node
const int ADC_BITS = 12;           // Uno R4 Minima ADC
int ADC_MAX = (1 << ADC_BITS) - 1;

float soilAlpha = 0.15f;           // EMA smoothing factor
float soilSmoothed = 0.0f;

int soilDryADC = -1;               // calibration (dry)
int soilWetADC = -1;               // calibration (wet)

// LoRa Constants ------------------------------------------------------------------------//
#define LORA_SERIAL Serial1 // Serial Port
static const uint32_t LoRaBaud = 9600; // GPS Baud Rate

// Sending Data constants
float insideTemp; 
float insideHum;
float gpsLat;
float gpsLon;
float soilMoisture;
float outsideTemp;

// Defining Device Types ----------------------------------------------------------------//
#define DHTTYPE    DHT11 

// Define Objects ------------------------------------------------------------------------//
DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
TinyGPSPlus gps; // The TinyGPSPlus object

// Software Serials
SoftwareSerial GPSSerial(GPSRXPin, GPSTXPin); // Serial for GPS object

// Custom Functions -----------------------------------------------------------------------------------------//
void saveCalibration() {  // Save SMS dry/wet values
  EEPROM.put(0, soilDryADC);
  EEPROM.put(sizeof(int), soilWetADC);
}

void loadCalibration() {  // Load previouse SMS dry/wet values
  EEPROM.get(0, soilDryADC);
  EEPROM.get(sizeof(int), soilWetADC);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){  // Map
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void gpsData() {  // GPS location data
  Serial.print(F("Location: ")); 
  if (gps.location.isValid()) {
    gpsLat = (gps.location.lat());
    gpsLon = (gps.location.lng());
    Serial.print(gpsLat, 6);
    Serial.print(F(","));
    Serial.print(gpsLon, 6);
  }
  else  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
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
  } else  {
    Serial.print(F("INVALID"));
  }

  Serial.println();

  // Print No. Satellites fixed
  Serial.print("Satellites: ");
  if (gps.satellites.isValid()) {
   Serial.println(gps.satellites.value());
  } else {
    Serial.println("INVALID");
  }

  Serial.println();
}

void smartDelay(unsigned long ms) { // Smart delay for GPS (constant feeding)
	unsigned long start = millis();
	do {
		while (GPSSerial.available()) {
			gps.encode(GPSSerial.read());
		}
	} while (millis() - start < ms);
}

void outTemp() {  // Outside thermistor 
  int analogValue = analogRead(thermistorPin);

  if (analogValue == 0) {
    Serial.println("Outside Temp: Error - analog value is 0");
    return;
  }

  // Convert analog reading to resistance of thermistor
  float thermistorResistance = seriesResistor * ((adcMax / analogValue) - 1);

  // Calculate temperature in Kelvin using Beta formula
  float temperatureK = 1.0 / ( (1.0 / (nominalTemperature + 273.15)) + 
                               (1.0 / betaCoefficient) * log(thermistorResistance / nominalResistance) );
  float temperatureC = temperatureK - 273.15;

  // Output to Serial Monitor
  Serial.print("Outside Temp: ");
  Serial.print(temperatureC, 1); // 1 decimal place
  Serial.println(" °C");
  outsideTemp = temperatureC;
}

void insideDht() {  // Inside DHT11 sensor
  // DHT11 ----------------------------------------------------------------------------------------------------------//
  // Get temperature event and print its value
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    insideTemp = (event.temperature);
    Serial.print(F("Temperature: "));
    Serial.print(insideTemp);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    insideHum = (event.relative_humidity);
    Serial.print(F("Humidity: "));
    Serial.print(insideHum);
    Serial.println(F("%"));
  }
}

float soilData() {  // SMS data
  int raw = analogRead(soilPin);
  Serial.print("Soil raw: "); Serial.println(raw);

  if (soilSmoothed == 0.0f) soilSmoothed = raw;
  soilSmoothed = soilAlpha * raw + (1.0f - soilAlpha) * soilSmoothed;

  if (soilDryADC >= 0 && soilWetADC >= 0 && soilDryADC != soilWetADC) {
    // Map regardless of whether wetADC > dryADC or not
    float pct = mapFloat(soilSmoothed, soilDryADC, soilWetADC, 0.0f, 100.0f);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
  } else {
    return -1.0f;  // not calibrated yet
  }
}

void setSoilDry() {
  soilDryADC = analogRead(soilPin);
  Serial.print("Dry calibration set: "); Serial.println(soilDryADC);
}

void setSoilWet() {
  soilWetADC = analogRead(soilPin);
  Serial.print("Wet calibration set: "); Serial.println(soilWetADC);
}

void sendLoRaAT(const String& cmd, unsigned long wait = 500) {  // Send initial LoRa AT commands (setup)
  LORA_SERIAL.println(cmd);
  Serial.print(">> "); Serial.println(cmd);

  // Give module time to respond
  delay(wait);

  // Optional: check response
  while (LORA_SERIAL.available()) {
    String resp = LORA_SERIAL.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.print("<< "); Serial.println(resp);
    }
  }
}

void sendLoRaData(const String& payload, unsigned long wait = 2000) { // Send data via LoRa
  // Build AT command with length and payload
  String cmd = "AT+SEND " + String(payload.length()) + "," + payload;

  // Send to RA-08H
  LORA_SERIAL.println(cmd);
  Serial.print(">> "); Serial.println(cmd);

  // Give module time to respond
  delay(wait);

  // Optional: check response
  while (LORA_SERIAL.available()) {
    String resp = LORA_SERIAL.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.print("<< "); Serial.println(resp);
    }
  }
}

void setup() {
  // Initialize devices
  Serial.begin(9600); // Physical Serial
  GPSSerial.begin(GPSBaud); // GPS Virtual Serial
  Wire.begin();
  dht.begin();

  // Set DHT11 sensor -----------------------------------------------------------------------------------------------------//
  sensor_t sensor;

  while (!Serial);

  // Soil Sensor set up --------------------------------------------------------------------------------//
  // Set ADC resolution explicitly for R4 Minima
  #if defined(analogReadResolution)
    analogReadResolution(ADC_BITS);
  #endif
  pinMode(soilPin, INPUT);

  loadCalibration(); // Load saved wet and dry moisture values
  
  // Initialize LoRa at 915 MHz (NZ band) --------------------------------------------------------------------------------//
  LORA_SERIAL.begin(9600); // RA-08H default baud is 9600
  delay(2000);
  Serial.println("Configuring RA-08H LoRa module...");
  sendLoRaAT("AT+CJOIN=1,0, 10,8");
  
  Serial.println("LoRa Module Configured");
}

void loop() {
  // Soil moisture Data --------------------------------------------------------------------------------------------------------//
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      setSoilDry();
      saveCalibration();
    } else if (c == 'w' || c == 'W') {
      setSoilWet();
      saveCalibration();
    }
  }
  
  float soilMoisture = soilData();

  if (soilMoisture >= 0.0f) {
    Serial.print("Moisture: ");
    Serial.print(soilMoisture, 1);
    Serial.println(" %");
  } else {
    Serial.println("Moisture: UNCALIBRATED");
  }

  // Inside Temp & Humidity ----------------------------------------------------------------------------------------------------------//
  insideDht();
  // Outside Thermisistor ----------------------------------------------------------------------------------------------------------//
  outTemp();
  // Get GPS Data --------------------------------------------------------------------------------------------------------------//
  gpsData();

  // Transmit Data via LoRa ----------------------------------------------------------------------------------------------------//
  // Build payload string
  String payload = "InTemp=" + String(insideTemp, 1) + "C"
                 + ",InHum=" + String(insideHum, 1) + "%"
                 + ",Lat=" + String(gpsLat, 6)
                 + ",Lon=" + String(gpsLon, 6)
                 + ",Soil=" + String(soilMoisture, 1) + "%"
                 + ",OutTemp=" + String(outsideTemp, 1) + "C";
  
  // Add AT send function to Payload string
  sendLoRaData(payload, 2000);

  // Delay between readings ---------------------------------------------------------------------------------------------------//
  smartDelay(10);

  // If No data is encoded to GPS module in 5s = Error
  if (millis() > 5000 && gps.charsProcessed() < 10) {
		Serial.println(F("No GPS data received: check wiring"));
	}
}
