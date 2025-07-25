// Including libraries
#include <inc.cpp>


SoftwareSerial LORA(9, 8); 
char val;

void setup() {
  Serial.begin(9600); 
  Serial.println("LORA is ready!");
  LORA.begin(9600);

  delay(100);
}

void loop() {
  if (Serial.available()) {
    val = Serial.read();
    LORA.print(val);
    delay(10);
  }

  if (LORA.available()) {
    val = LORA.read();
    Serial.print(val);
    delay(10);
  }
}

// // Constants ------------------------------------------------------------------------//
// #define DHTPIN 12  // DHT11 Pin

// // Grove GPS Constants
// static const int GPSRXPin = 2, GPSTXPin = 3;  // Serial Port
// static const uint32_t GPSBaud = 9600; // GPS Baud Rate
// const long interval = 10000;  
// unsigned long previousMillis = 0;

// // Thermistor Constants
// const int thermistorPin = A0; // Thermistor pins
// const float seriesResistor = 10000.0;  // 10k Ohm series resistor
// const float nominalResistance = 10000.0; // Resistance of thermistor at 25ºC
// const float nominalTemperature = 25.0;   // Nominal temperature (ºC)
// const float betaCoefficient = 3892.0;    // Beta coefficient of the thermistor
// const float adcMax = 1023.0;             // Max value from analogRead

// // LoRa Constants
// static const int LoRaRXPin = 9, LoRaTXPin = 8; // Serial Port
// static const uint32_t LoRaBaud = 9600; // GPS Baud Rate
// char val;

// // Defining Device Types ----------------------------------------------------------------//
// #define DHTTYPE    DHT11 

// // Define Objects ------------------------------------------------------------------------//
// DHT_Unified dht(DHTPIN, DHTTYPE); //Create dht object
// TinyGPSPlus gps; // The TinyGPSPlus object

// // Software Serials
// SoftwareSerial GPSSerial(GPSRXPin, GPSTXPin); // Serial for GPS object
// SoftwareSerial LoRaSerial(LoRaRXPin, LoRaTXPin);  // Serial for LoRa object

// // Custom Functions -----------------------------------------------------------------------------------------//
// void displayInfo() {
//   Serial.print(F("Location: ")); 
//   if (gps.location.isValid()) {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   }
//   else  {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid()) {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else  {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid()) {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   } else  {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
// }

// // This custom version of delay() ensures that the gps object is being "fed"
// void smartDelay(unsigned long ms) {
// 	unsigned long start = millis();
// 	do {
// 		while (GPSSerial.available()) {
// 			gps.encode(GPSSerial.read());
// 		}
// 	} while (millis() - start < ms);
// }

// void outTemp() {
//   int analogValue = analogRead(thermistorPin);

//   if (analogValue == 0) {
//     Serial.println("Outside Temp: Error - analog value is 0");
//     return;
//   }

//   // Convert analog reading to resistance of thermistor
//   float voltage = analogValue / adcMax; // fraction of Vcc
//   float thermistorResistance = seriesResistor * ((adcMax / analogValue) - 1);

//   // Calculate temperature in Kelvin using Beta formula
//   float temperatureK = 1.0 / ( (1.0 / (nominalTemperature + 273.15)) + 
//                                (1.0 / betaCoefficient) * log(thermistorResistance / nominalResistance) );
//   float temperatureC = temperatureK - 273.15;

//   // Output to Serial Monitor
//   Serial.print("Outside Temp: ");
//   Serial.print(temperatureC, 1); // 1 decimal place
//   Serial.println(" °C");
// }

// void setup() {
//   // Initialize devices
//   Serial.begin(9600); // Physical Serial
//   GPSSerial.begin(GPSBaud); // GPS Virtual Serial
//   LoRaSerial.begin(LoRaBaud); // LoRa Virtual Serial
//   Wire.begin();
//   dht.begin();

//   // Set DHT11 sensor -----------------------------------------------------------------------------------------------------//
//   sensor_t sensor;

// }

// void loop() {
//   // DHT11 ----------------------------------------------------------------------------------------------------------//
//   // Get temperature event and print its value
//   sensors_event_t event;

//   dht.temperature().getEvent(&event);
//   if (isnan(event.temperature)) {
//     Serial.println(F("Error reading temperature!"));
//   }
//   else {
//     Serial.print(F("Temperature: "));
//     Serial.print(event.temperature);
//     Serial.println(F("°C"));
//   }
//   // Get humidity event and print its value.
//   dht.humidity().getEvent(&event);
//   if (isnan(event.relative_humidity)) {
//     Serial.println(F("Error reading humidity!"));
//   }
//   else {
//     Serial.print(F("Humidity: "));
//     Serial.print(event.relative_humidity);
//     Serial.println(F("%"));
//   }
//   // Outside Thermisistor ----------------------------------------------------------------------------------------------------------//
//   outTemp();

//   // GPS ----------------------------------------------------------------------------------------------------------//
//   // Print No. Satellites fixed
//   Serial.print("Satellites: ");
//   if (gps.satellites.isValid()) {
//    Serial.println(gps.satellites.value());
//   } else {
//     Serial.println("INVALID");
//   }

//   displayInfo();

//   // Transmit Data via LoRa ----------------------------------------------------------------------------------------------------//

//   // Delay between readings ---------------------------------------------------------------------------------------------------//
//   smartDelay(10);

//   // If No data is encoded to GPS module in 5s = Error
//   if (millis() > 5000 && gps.charsProcessed() < 10) {
// 		Serial.println(F("No GPS data received: check wiring"));
// 	}
// }
