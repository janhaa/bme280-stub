
/*
BME280 I2C Test.ino
This code shows how to record data from the BME280 environmental sensor
using I2C interface. This file is an example file, part of the Arduino
BME280 library.
GNU General Public License
Written: Dec 30 2015.
Last Updated: Oct 07 2017.
Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
 */

#include <BME280I2C.h>
#include <Wire.h>
#include <ESP8266WiFi.h>

#define SERIAL_BAUD 115200
#define D5 14 

int fanPin = 14;
int dutyCycle = 0;
int fanspeed = 0;
float temp_now;
float hum_now;

BME280I2C::Settings settings (
  BME280::OSR_X1, // Temperature
  BME280::OSR_X1, // Humidity
  BME280::OSR_X1, // Pressure
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_Off,
  BME280::SpiEnable_False,
  0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

//////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(SERIAL_BAUD);

  while(!Serial) {} // Wait

  Wire.begin();

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  pinMode(fanPin, OUTPUT); // sets the pins as outputs:
  
  analogWriteRange(100); // to have a range 1 - 100 for the fan
  analogWriteFreq(10000);

}

void controlFanSpeed (int fanSpeedPercent) {
  Serial.print("Fan Speed: ");
  Serial.print(fanSpeedPercent);
  Serial.println("%");  
  analogWrite(fanPin, fanSpeedPercent); // set the fan speed
}

void updateSensorTemp() {
  temp_now = bme.temp();
  Serial.print("Temperature: ");
  Serial.print(temp_now);
  Serial.println(" °C");
}
void updateSensorHum() {
  hum_now = bme.hum();
  Serial.print("Humidity: ");
  Serial.print(hum_now);
  Serial.println(" %");
}

//////////////////////////////////////////////////////////////////
void loop()
{
   updateSensorTemp();
   updateSensorHum();
   if(temp_now >= 25 || hum_now >= 80){
    fanspeed=100;
   }
   else
   {
    fanspeed=0;
   }
   controlFanSpeed(fanspeed); 
   delay(10000);
}