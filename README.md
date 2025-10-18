# 13DIT-Electronics
<----- Wireless Soil Moisture Sensor ----->

Created by: 
Benjamin Beard - 22081
2025
Hauraki Plains College
AS91904
AS91907

System Proccesses:
This system is desinged to wirelessly test the soil moisture content on soil, along with its temperature. The system also collects the current GPS coordinates along with tracking humidity and temperature inside the system. All of this data is wirelessly communicated via LoRa point-2-point protocol to a reciever module. 

Components:
- Arduino Uno R4 Minima https://docs.arduino.cc/hardware/uno-r4-minima/ 
- 2x Crowtail LoRa Ra-08H https://www.elecrow.com/crowtail-lora-ra-08h-for-long-range-communication-803-930mhz.html?srsltid=AfmBOoqkccuuLjzD_eBHyi32loE0o96iN14Qucd1nBNX8ooI8jwvbC7D 
- GPS https://www.digikey.co.nz/en/products/detail/seeed-technology-co-ltd/109020022/12323450
- DHT11 https://www.digikey.co.nz/en/products/detail/seeed-technology-co-ltd/101020011/5482602
- NTC 10KOhm Thermistor https://www.digikey.co.nz/en/products/detail/ei-sensor-technologies/ETP10002/15790993
- LM2596S Buck Converter https://www.digikey.co.nz/en/products/detail/dfrobot/DFR0379/7087190
- 3s 18650 BMS https://www.digikey.co.nz/en/products/detail/dfrobot/FIT0869/15997370 
- Custom TLC555CP moisture module
- 3x Li-ion 18650 Batteries
- Green / Red LED
- Power Switch
- 2x Push Button switches

<--- TESTING SYSTEM --- >

Step: Configuring PC
1) Install puTTY for serial communication with LoRa-Ra-08H module.
puTTY: https://putty.org/index.html 

2) Connect LoRa-Ra-08H module to PC via TTL serial-usb adapter. 
!IMPORTANT:
    (Make sure TTL is set to +5.0V not 3.3V)
    (Make sure LoRa-08H is not set to Boot mode via switch)

PINS:
Ra-08H RX    -> TTL TX
Ra-08H TX    -> TTL RX
Ra-08H VCC -> TTL VCC
Ra-08H GND -> TTL GND

3) Open puTTY and open ‘serial’ in menu and configure serial communication settings.
    Speed: 9600
    Data bits: 8
    Stop bits: 1
    Parity: none
    Flow control: none or XON/XOFF
- Click ‘open’

Step: Configuring Wireless Soil Moisture Sensor. 
4) Turn on power switch. Red or Green LED should show. 
    Red = No Moisture calibration set.
    Green = Moisture calibration set.

5) Wireless LoRa system should automatically connect to reciever module if within range, 1-2km (without major obstruction like buildings or trees.)

Step: Connecting to arduino via USB-C Cable. 
6) Insert USB-C cable through open port on the top of the system. Configure puTTY the same as for the reciever LoRa module and open. Turn on arduino system via switch. 
