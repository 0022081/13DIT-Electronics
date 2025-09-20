// Including libraries
#include <header.h>

// Constants ------------------------------------------------------------------------//
#define DHTPIN 12  // DHT11 Pin

// Grove GPS Constants
static const int GPSRXPin = 2, GPSTXPin = 3;  // Serial Port
static const uint32_t GPSBaud = 9600; // GPS Baud Rate
const long interval = 10000;  
unsigned long previousMillis = 0;

// Thermistor Constants
const int thermistorPin = A0; // Thermistor pins
const float seriesResistor = 10000.0;  // 10k Ohm series resistor
const float nominalResistance = 10000.0; // Resistance of thermistor at 25ºC
const float nominalTemperature = 25.0;   // Nominal temperature (ºC)
const float betaCoefficient = 3892.0;    // Beta coefficient of the thermistor
const float adcMax = 1023.0;             // Max value from analogRead

// Soil Data Constants
const uint8_t sensorPin = 2;    // digital input from TLC555 pin3
const unsigned long pulseTimeout = 300000UL; // µs (300 ms) timeout for pulseIn

// Calibration values (replace with measured values for your soil probe)
float dryFreq = 95000.0;   // Hz when dry
float wetFreq = 45000.0;   // Hz when saturated

// Smoothing
const float alpha = 0.2;
float smoothedFreq = 0.0;

// LoRa Constants
static const int LoRaRXPin = 9, LoRaTXPin = 8; // Serial Port
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
#define LORA_SERIAL Serial1

// Define Objects ------------------------------------------------------------------------//
DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
TinyGPSPlus gps; // The TinyGPSPlus object

// Software Serials
SoftwareSerial GPSSerial(GPSRXPin, GPSTXPin); // Serial for GPS object

// Custom Functions -----------------------------------------------------------------------------------------//
void gpsData() {
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

// Ensures that the gps object is being "fed"
void smartDelay(unsigned long ms) {
	unsigned long start = millis();
	do {
		while (GPSSerial.available()) {
			gps.encode(GPSSerial.read());
		}
	} while (millis() - start < ms);
}

void outTemp() {
  int analogValue = analogRead(thermistorPin);

  if (analogValue == 0) {
    Serial.println("Outside Temp: Error - analog value is 0");
    return;
  }

  // Convert analog reading to resistance of thermistor
  float voltage = analogValue / adcMax; // fraction of Vcc
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

void insideDht() {
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

float soilData() {
  // Measure HIGH and LOW pulse times
  unsigned long highUs = pulseIn(sensorPin, HIGH, pulseTimeout);
  unsigned long lowUs  = pulseIn(sensorPin, LOW,  pulseTimeout);

  // Timeout guard
  if (highUs == 0 || lowUs == 0) {
    Serial.println("No valid pulses (timeout). Check wiring or sensor power.");
    return -1.0; // return invalid value
  }

  unsigned long periodUs = highUs + lowUs;
  float freqHz = (periodUs > 0) ? (1000000.0f / (float)periodUs) : 0.0f;

  // Initialize smoothing
  if (smoothedFreq == 0.0f) smoothedFreq = freqHz;
  smoothedFreq = alpha * freqHz + (1.0f - alpha) * smoothedFreq;

  // Map frequency to % moisture
  float moisturePct = (dryFreq != wetFreq) ?
    (smoothedFreq - dryFreq) / (wetFreq - dryFreq) * 100.0f : 0.0f;

  // Clamp
  if (moisturePct < 0.0f) moisturePct = 0.0f;
  if (moisturePct > 100.0f) moisturePct = 100.0f;

  // Debug print
  Serial.print("HIGH_us: "); Serial.print(highUs);
  Serial.print(" | LOW_us: "); Serial.print(lowUs);
  Serial.print(" | Period_us: "); Serial.print(periodUs);
  Serial.print(" | Freq: "); Serial.print(freqHz, 1);
  Serial.print(" Hz (smoothed: "); Serial.print(smoothedFreq, 1); Serial.print(" Hz)");
  Serial.print(" | Moisture: "); Serial.print(moisturePct, 1); Serial.println(" %");

  return moisturePct;
}

void sendLoRaData(String payload, unsigned long wait = 500) {
  LORA_SERIAL.println(payload);
  delay(wait);

  // Read back any response from module
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
  
  // Initialize LoRa at 915 MHz (NZ band) --------------------------------------------------------------------------------//
  LORA_SERIAL.begin(9600); // RA-08H default baud is 9600
  delay(2000);
  Serial.println("Configuring RA-08H LoRa module...");
  sendLoRaData("AT+CGSN?");// Query firmware just to check connection
  sendLoRaData("AT+MODE=0");// Set LoRa mode
  sendLoRaData("AT+FREQ=915000000");// Set frequency to 915 MHz
  // Set LoRa parameters:
  // AT+PARAMETER=<SpreadFactor>,<Bandwidth>,<CodingRate>,<PreambleLength>
  // SpreadFactor 12, BW=7 (125kHz), CR=1 (4/5), Preamble=8
  sendLoRaData("AT+PARAMETER=12,7,1,8");
  Serial.println("LoRa Module Configured");

  pinMode(sensorPin, INPUT);
}

void loop() {
  // Inside Temp & Humidity ----------------------------------------------------------------------------------------------------------//
  insideDht();
  // Outside Thermisistor ----------------------------------------------------------------------------------------------------------//
  outTemp();
  // Get GPS Data --------------------------------------------------------------------------------------------------------------//
  gpsData();
  
  // Soil moisture Data --------------------------------------------------------------------------------------------------------//
  float soilMoisture = soilData(); // Call your function

  if (soilMoisture >= 0.0) {
    Serial.print("Moisture % (usable value): ");
    Serial.println(soilMoisture, 1);
  }

  // Transmit Data via LoRa ----------------------------------------------------------------------------------------------------//
  // Build payload string
  String payload = "InTemp=" + String(insideTemp, 1) + "C"
                 + ",InHum=" + String(insideHum, 1) + "%"
                 + ",Lat=" + String(gpsLat, 6)
                 + ",Lon=" + String(gpsLon, 6)
                 + ",Soil=" + String(soilMoisture, 1) + "%"
                 + ",OutTemp=" + String(outsideTemp, 1) + "C";
  
  Serial.println(payload);
  // Add AT send function to Payload string
  String cmd = "AT+SEND=" + String(payload.length()) + "," + payload;
  sendLoRaData(cmd, 2000);

  // Delay between readings ---------------------------------------------------------------------------------------------------//
  smartDelay(10);

  // If No data is encoded to GPS module in 5s = Error
  if (millis() > 5000 && gps.charsProcessed() < 10) {
		Serial.println(F("No GPS data received: check wiring"));
	}
}
