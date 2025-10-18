// Including libraries
#include <header.h>

// Constants ---------------------------------------------------------------------------//
#define DHTPIN 10  // DHT11 Pin

// Grove GPS Constants
static const int GPSRXPin = 2, GPSTXPin = 4;  // Serial Port
static const uint32_t GPSBaud = 9600;         // GPS Baud Rate
const long interval = 10000;  
unsigned long previousMillis = 0;

// Thermistor Constants ------------------------------------------------------------------//
const int thermistorPin = A2;            // Thermistor pins
const float seriesResistor = 10000.0;    // 10k Ohm series resistor
const float nominalResistance = 10000.0; // Resistance of thermistor at 25ºC
const float nominalTemperature = 25.0;   // Nominal temperature (ºC)
const float betaCoefficient = 3892.0;    // Beta coefficient of the thermistor
const float VREF = 3.3f;                 // ADC reference (3.3V)
float t_offset = -15.8;                  // calibration offset

// Soil Data Constants ------------------------------------------------------------------//
const uint8_t soilPin = A1;        // analog pin for envelope node
const int ADC_BITS = 14;           // Uno R4 Minima ADC
int ADC_MAX = (1 << ADC_BITS) - 1; // set ADC from measurement bits

float soilAlpha = 0.15f;           // EMA smoothing factor
float soilSmoothed = 0.0f;        

int soilDryADC = -1;               // calibration (dry)
int soilWetADC = -1;               // calibration (wet)

// LoRa Constants ------------------------------------------------------------------------//
int counterLoRa = 0;              // payload sending number

// Buttons ----------------------------- -------------------------------------------------//
#define BUTTON_WET_PIN 6    // Wet button PIN
#define BUTTON_DRY_PIN 7    // Dry button PIN

// Calibration LEDs
const int redLED = 9;       // Red LED PIN
const int greenLED = 8;     // Green LED PIN

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
bool halfHourCheck = (checkMinute == 0 || checkMinute == 30); // interval for testing every 1/2 hr
int lastTriggerHour = -1;
int lastTriggerMinute = -1;

// Soil module state
bool testModuleState = false; 

// Using onboard Millis
unsigned long currentMillis = 0;          // stores value of millis()
unsigned long previouseOnBoardMillis = 0; // stores last board run time
unsigned long interval_duration = 1000;   // testing duration

bool breakTest = false; // break testing during loop if returns true

// Defining Device Types ----------------------------------------------------------------//
#define DHTTYPE    DHT11 

// Define Objects ------------------------------------------------------------------------//
DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
TinyGPSPlus gps;                  // The TinyGPSPlus object
Button2 buttonWet, buttonDry;     // calibration buttons

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
void gpsData() {  
  Serial.print(F("Location: ")); // GPS location
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

  Serial.print(F("  Date/Time: ")); // GPS date & time
  if (gps.date.isValid()) {
    Serial.print(gps.date.month()); // print date to serial monitor
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));     // print time to serial monitor
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
  Serial.print(F("Satellites: "));  
  if (gps.satellites.isValid()) {
   Serial.println(gps.satellites.value());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

// Return interval for testing delays
bool checkTestingTime() {
  // get time from gps module
  if (gps.time.isValid()) {
    int checkHour = gps.time.hour();
    int checkMinute = gps.time.minute();
  } else {
    Serial.println("Check Time Failed!");
  }

  // every half an hour run test
  if (halfHourCheck && (checkMinute != lastTriggerMinute || checkHour != lastTriggerHour)) {
    lastTriggerHour = checkHour;
    lastTriggerMinute = checkMinute;
    return true;
  } else {
    return false;
  }
  
}

// // Return interval for testing every 5 seconds
// bool checkTestingTime() {
//   static int lastTriggerSecond = -1;

//   if (gps.time.isValid()) {
//     int currentSecond = gps.time.second();

//     // Check if 5 seconds have passed since last trigger
//     if (lastTriggerSecond == -1 || (currentSecond - lastTriggerSecond + 60) % 60 >= 5) {
//       lastTriggerSecond = currentSecond;
//       return true;
//     }

//   } else {
//     Serial.println(F("Check Time Failed!")); // debug
//   }

//   return false;
// }

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
  int adc = analogRead(thermistorPin);
  if (adc <= 0) return -273.15f;                     // avoid div by zero
  if (adc >= ADC_MAX) return 150.0f;                 // sensor saturated

  float vout = (float)adc / (float)ADC_MAX * VREF;    // convert ADC to voltage (Vout)

  float denom = (VREF - vout);
  if (denom <= 0.0f) return 150.0f;                        // safety check: denominator must be >0
  float rTherm = seriesResistor * (vout / (VREF - vout));  // compute thermistor resistance (R_therm)

  // compute temperature using Beta equation
  float t0 = nominalTemperature + 273.15f;           // T0 in Kelvin
  float invT = (1.0f / t0) + (1.0f / betaCoefficient) * log(rTherm / nominalResistance);
  if (invT <= 0.0f) return -273.15f;                 // safety
  float tKelvin = 1.0f / invT;

  // convert to Celsius
  float tempC = tKelvin - 273.15f;        
  float actual_temp = tempC + t_offset;   // apply calibration offset
  return actual_temp;                     // return temperature to payload
}

// Read inside Temp & Humidity
void insideDht(float &temp, float &hum) {
  // DHT11 ----------------------------------------------------------------------------------------------------------//
  // Get temperature 
  sensors_event_t event;

  dht.temperature().getEvent(&event); // get inside temperature
  if (isnan(event.temperature)) {
    temp = NAN; // return inavlid if cant read
  }
  else {
    temp = (event.temperature);
  }
  // Get humidity
  dht.humidity().getEvent(&event);  // get inside humidity
  if (isnan(event.relative_humidity)) {
    hum = NAN;  // return invalid if cant read
  }
  else {
    hum = (event.relative_humidity);    
  }
}

// Return soil moisture % value
float soilData() { 
  int raw = analogRead(soilPin);
  //Serial.print("Soil raw: "); Serial.println(raw);  // print raw soil data

  // Smooth soil data from high fluctuations
  if (soilSmoothed == 0.0f) soilSmoothed = raw;
  soilSmoothed = soilAlpha * raw + (1.0f - soilAlpha) * soilSmoothed;

  // apply calibrations to find %
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
    //Serial.println(F("Wet Button Pressed")); // debug
    setSoilWet();
    saveCalibration();
  } else if (btn == buttonDry) {
    //Serial.println(F("Dry button pressed")); // debug
    setSoilDry();
    saveCalibration();
  }
}

// Send LoRa packets
void sendLoRaData(const String &payload) { // Send data via LoRa
  
  // create payload in bytes
  const char *sendData = payload.c_str();
  size_t payloadLength = strlen(sendData);

  // send payload through TX and RX pins
  Serial1.write((const uint8_t*)sendData, payloadLength);
  
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
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, HIGH);

  Serial.begin(9600); // Physical Serial
  delay(50);

  Serial1.begin(9600);      // lora serial
  GPSSerial.begin(GPSBaud); // GPS virtual serial
  Wire.begin();             // dht11 sensor
  dht.begin();

  analogReadResolution(ADC_BITS);   // Set analog read resolution to 14 bits
  
  // Set button handlers and Pins
  buttonWet.begin(BUTTON_WET_PIN); 
  buttonWet.setTapHandler(click);
  buttonDry.begin(BUTTON_DRY_PIN);
  buttonDry.setTapHandler(click);

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

  //Serial.println("Testing Soil Data"); // debug

  // Soil moisture Data --------------------------------------------------------------------------------------------------------//
  // Check for soil value error, if not, set value
  soilMoisture = soilData();
  if (soilMoisture >= 0.0f) {
    Serial.print("Moisture: ");
    Serial.print(soilMoisture, 1);
    Serial.println(" %");
    digitalWrite(greenLED, HIGH); // set LEDS if calibrated correctly
    digitalWrite(redLED, LOW);
  } else {
    Serial.println("Moisture: UNCALIBRATED"); // print error if uncalibrated
    digitalWrite(greenLED, LOW);              // set LEDS if calibrated wrong
    digitalWrite(redLED, HIGH);
  }

  // Inside Temp & Humidity ----------------------------------------------------------------------------------------------------------//
  // Check for inside temp & hum value error, if not, set values
  insideDht(insideTemp, insideHum);
  if (!isnan(insideTemp)) {
    Serial.print(F("Inside Temp: "));
    Serial.print(insideTemp);
    Serial.println(F("°C"));
  } else {
    Serial.println(F("Inside Temp: INVALID"));
  }

  if (!isnan(insideHum)) {
    Serial.print(F("Humidity: "));
    Serial.print(insideHum);
    Serial.println(F("%"));
  } else {
    Serial.println(F("Inside Hum: INVALID"));
  }

  // Outside Thermisistor ----------------------------------------------------------------------------------------------------------//
  // Cehck for outside temp value error, if not, set value
  outsideTemp = outTemp();
  if (!std::isnan(outsideTemp) && !std::isinf(outsideTemp) || outsideTemp <= -273.15 || outsideTemp >= 150) {
    Serial.print(F("Out Temp: "));
    Serial.print(outsideTemp, 2);
    Serial.println();
    } else {
      Serial.print(F("Outside Temp: INVALID"));
    }
  
  // Get GPS Data --------------------------------------------------------------------------------------------------------------//
  gpsData();

  // Transmit Data via LoRa ----------------------------------------------------------------------------------------------------//
  // Build payload string
  String payload = "InTemp=" + String(insideTemp, 1) + "C"
                 + ",InHum=" + String(insideHum, 1) + "%"
                 + ",Lat=" + String(gpsLat, 6)
                 + ",Lon=" + String(gpsLon, 6)
                 + ",Soil=" + String(soilMoisture, 1) + "%"
                 + ",OutTemp=" + String(outsideTemp, 1) + "C"
                 + "\n";  // end newline

  // Send soil data via LoRa
  sendLoRaData(payload);
  delay(1000); // delay before checking for response
  // Reciever any responses from receiver module
  receiveLoRaData();
  delay(200); // delay between readings
}

// Main board system loop
void loop() {

  currentMillis = millis(); // set current millis

  // Check for Soil Moisture Calibration via buttons
  buttonWet.loop();
  buttonDry.loop();

  // Check time for testing 
  if (checkTestingTime() == true) {
    //Serial.println("Testing invertal expired, testing..."); // debug
    for (int i = 1; i <= 5; i++) {
      if (breakTest == true) {
        break;
      }
      updateSoilMoistureData();
      Serial.println("Test: " + i);
    }
  } else if (checkTestingTime() == false) {
    //Serial.println("Testing interval not expired"); // debug
  }

  // Delay and encoding for GPS
  smartDelay(10);

  // If No data is encoded to GPS module in 5s = Error
  if (millis() > 5000 && gps.charsProcessed() < 10) {
		//Serial.println("No GPS data received: INVALID WIRING");
	}
}