// Including libraries
#include <header.h>

// Constants ---------------------------------------------------------------------------//
#define DHTPIN 10  // DHT11 Pin

// Grove GPS Constants
static const int GPSRXPin = 2, GPSTXPin = 4;  // Serial Port
static const uint32_t GPSBaud = 9600; // GPS Baud Rate
const long interval = 10000;  
unsigned long previousMillis = 0;

// Thermistor Constants ------------------------------------------------------------------//
const int thermistorPin = A2; // Thermistor pins
const float seriesResistor = 10000.0;  // 10k Ohm series resistor
const float nominalResistance = 10000.0; // Resistance of thermistor at 25ºC
const float nominalTemperature = 25.0;   // Nominal temperature (ºC)
const float betaCoefficient = 3892.0;    // Beta coefficient of the thermistor
const float VREF = 3.26f;                  // ADC reference (3.3V)

// Soil Data Constants ------------------------------------------------------------------//
const uint8_t soilPin = A1;        // analog pin for envelope node
const int ADC_BITS = 14;           // Uno R4 Minima ADC
int ADC_MAX = (1 << ADC_BITS) - 1;

float soilAlpha = 0.15f;           // EMA smoothing factor
float soilSmoothed = 0.0f;

int soilDryADC = -1;               // calibration (dry)
int soilWetADC = -1;               // calibration (wet)

// LoRa Constants ------------------------------------------------------------------------//
int counterLoRa = 0;

// Buttons ------------------------------------------------------------------------------//
#define BUTTON_WET_PIN 6
#define BUTTON_DRY_PIN 7

// Sending Data constants
float insideTemp; 
float insideHum;
float gpsLat;
float gpsLon;
float soilMoisture;
float outsideTemp;

// Code running time constants ---------------------------------------------------------//
float checkMinute;
float checkHour;
bool halfHourCheck = (checkMinute == 0 || checkMinute == 30);
int lastTriggerHour = -1;
int lastTriggerMinute = -1;

// Soil module state
bool testModuleState = false;

unsigned long currentMillis = 0; // stores value of millis()
unsigned long previouseOnBoardMillis = 0; // stores last board run time

bool breakTest = false; // break testing during loop if returns true

// Defining Device Types ----------------------------------------------------------------//
#define DHTTYPE    DHT11 

// Define Objects ------------------------------------------------------------------------//
DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
TinyGPSPlus gps; // The TinyGPSPlus object
Button2 buttonWet, buttonDry;

// Software Serials
SoftwareSerial GPSSerial(GPSRXPin, GPSTXPin); // Serial for GPS object

// Custom Functions -----------------------------------------------------------------------------------------//
// Save soil data
void saveCalibration() {  // Save SMS dry/wet values
  EEPROM.put(0, soilDryADC);
  EEPROM.put(sizeof(int), soilWetADC);
}

// Load saved soil data
void loadCalibration() {  // Load previouse SMS dry/wet values
  EEPROM.get(0, soilDryADC);
  EEPROM.get(sizeof(int), soilWetADC);
}

// Soil data smoothing func.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){  // Map
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Read GPS location & time
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

// Return interval for testing delays
bool checkTestingTime() {
  if (gps.time.isValid()) {
    int checkHour = gps.time.hour();
    int checkMinute = gps.time.minute();
  } else {
    Serial.println("Check Time Failed!");
  }

  if (halfHourCheck && (checkMinute != lastTriggerMinute || checkHour != lastTriggerHour)) {
    lastTriggerHour = checkHour;
    lastTriggerMinute = checkMinute;
    return true;
  } else {
    return false;
  }
}

// Continiouse GPS encoding with delay
void smartDelay(unsigned long ms) { // Smart delay for GPS (constant feeding)
	unsigned long start = millis();
	do {
		while (GPSSerial.available()) {
			gps.encode(GPSSerial.read());
		}
	} while (millis() - start < ms);
}

// Return outside Temp
float outTemp() {
  int adc = analogRead(thermistorPin);               // 0..4095
  if (adc <= 0) return -273.15f;                     // avoid div by zero, return nonsense cold
  if (adc >= ADC_MAX) return 150.0f;                  // sensor saturated; return large temp (or handle differently)

  float vout = (float)adc / (float)ADC_MAX * VREF;  // convert ADC to voltage (Vout)

  float denom = (VREF - vout);
  if (denom <= 0.0f) return 150.0f;                  // safety check: denominator must be >0
  float rTherm = seriesResistor * (vout / denom);   // compute thermistor resistance (R_therm)

  // compute temperature using Beta equation
  float t0 = nominalTemperature + 273.15f;           // T0 in Kelvin
  float invT = (1.0f / t0) + (1.0f / betaCoefficient) * log(rTherm / nominalResistance);
  if (invT <= 0.0f) return -273.15f;                 // safety
  float tKelvin = 1.0f / invT;

  // convert to Celsius
  float tempC = tKelvin - 273.15f;
  Serial.print("Out Temp: ");
  Serial.print(tempC, 2);
  Serial.println("");
  return tempC;
}

// Read inside Temp & Humidity
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

// Return soil moisture % value
float soilData() { 
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

// Set dry soil calibration
void setSoilDry() {
  soilDryADC = analogRead(soilPin);
  Serial.print("Dry calibration set: "); Serial.println(soilDryADC);
}

// Set wet soil calibration 
void setSoilWet() {
  soilWetADC = analogRead(soilPin);
  Serial.print("Wet calibration set: "); Serial.println(soilWetADC);
}

// Check button debouce func.
void click(Button2& btn) {
  if (btn == buttonWet) {
    setSoilWet();
    saveCalibration();
    Serial.println("Wet Button Pressed");
  } else if (btn == buttonDry) {
    setSoilDry();
    saveCalibration();
    Serial.println("Dry button pressed");
  }
}

// Send LoRa packets
void sendLoRaData(const String &payload) { // Send data via LoRa
  
  // create payload in bytes
  const char *sendData = payload.c_str();
  size_t payloadLength = strlen(sendData);

  // send payload through TX and RX pins
  Serial1.write((const uint8_t*)sendData, payloadLength);
  Serial1.write('\n');
  
  // Print sent payload to serial monitor
  Serial.println(sendData);
}

// Receive LoRa packets
void receiveLoRaData() {
  // read packet header bytes:
  String incoming = "";

  while (Serial1.available()) {
    incoming += (char)Serial1.read();
  }

  Serial.println("Message: " + incoming);
}

// System set up
void setup() {
  // Initialize devices
  Serial.begin(9600); // Physical Serial
  delay(50);

  Serial1.begin(9600);
  GPSSerial.begin(GPSBaud); // GPS Virtual Serial
  Wire.begin();
  dht.begin();

  analogReadResolution(ADC_BITS); // Set analog read resolution to 14 bits
  buttonWet.begin(BUTTON_WET_PIN);
  buttonWet.setClickHandler(click);
  buttonDry.begin(BUTTON_DRY_PIN);
  buttonDry.setClickHandler(click);

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
}

// Test Soil Data and Temperature Loop
void updateSoilMoistureData () {

  Serial.println("Testing Soil Data");

  // Soil moisture Data --------------------------------------------------------------------------------------------------------//
  // Set soil moisture value
  float soilMoisture = soilData();

  // Check soil moisture calibration values - that calibration was set
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
  outsideTemp = outTemp();
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
  
  // Send soil data via LoRa
  sendLoRaData(payload);
  delay(200); // delay before checking for response
  // Reciever any responses from receiver module
  receiveLoRaData();
  delay(100); // delay between readings
}

// Main board system loop
void loop() {
  
  // Check for Soil Moisture Calibration via buttons
  buttonWet.loop();
  buttonDry.loop();

  // Check time for testing 
  if (checkTestingTime() == true) {
    Serial.println("Testing invertal expired, testing...");
    for (int i = 1; i <= 5; i++) {
      if (breakTest == true) {
        break;
      }
      updateSoilMoistureData();
      Serial.println("Test: " + i);
    }
  } else if (checkTestingTime() == false) {
    Serial.println("Testing interval not expired");
  }

  // Delay and encoding for GPS
  smartDelay(10);

  // If No data is encoded to GPS module in 5s = Error
  if (millis() > 5000 && gps.charsProcessed() < 10) {
		Serial.println("No GPS data received: check wiring");
	}
}