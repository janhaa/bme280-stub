#include <BME280.h>
#include <BME280I2C_BRZO.h>
#include <BME280I2C.h>
#include <BME280Spi.h>
#include <BME280SpiSw.h>
#include <EnvironmentCalculations.h>


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

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char *ssid = "Paketshop";
const char *password = "Lololol520";

#define SERIAL_BAUD 115200
#define D5 14

int dutyCycle = 0;
int fanspeed = 0;
float temp_now;
float hum_now;

// the number of the LED pin
const int fanPin = 27; // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int fanPWMChannel = 0;
const int resolution = 8;

BME280I2C::Settings settings(
    BME280::OSR_X1, // Temperature
    BME280::OSR_X1, // Humidity
    BME280::OSR_X1, // Pressure
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_Off,
    BME280::SpiEnable_False,
    0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings); // Default : forced mode, standby time = 1000 ms
                         // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

//////////////////////////////////////////////////////////////////

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void setupFan()
{
  // configure LED PWM functionalitites
  ledcSetup(fanPWMChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(fanPin, fanPWMChannel);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);

  while (!Serial)
  {
  } // Wait

  connectToWiFi();

  Wire.begin();

  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch (bme.chipModel())
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

  setupFan();
}

void controlFanSpeed(int fanSpeedPercent)
{
  Serial.print("Fan Speed: ");
  Serial.print(fanSpeedPercent);
  Serial.println("%");

  ledcWrite(fanPWMChannel, pow(2, resolution) - 1);
}

void updateSensorTemp()
{
  temp_now = bme.temp();
  Serial.print("Temperature: ");
  Serial.print(temp_now);
  Serial.print(" °C ");
}
void updateSensorHum()
{
  hum_now = bme.hum();
  Serial.print("Humidity: ");
  Serial.print(hum_now);
  Serial.print(" % ");
}

void publishValues()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("error publishing values: no connection to wifi!");
    return;
  }

  String body = "{\"id\":\"igor\",\"humidity\":\"";
  body += hum_now;
  body += "\", \"temperature\": \"";
  body += temp_now;
  body += "\"}";

  HTTPClient http;
  http.begin("https://1src.tech/api/produce/bme280");
  http.addHeader("Content-Type", "application/json");
  http.POST(body);

  temp_now = bme.temp();
  hum_now = bme.hum();
}

//////////////////////////////////////////////////////////////////
void loop()
{
  updateSensorTemp();
  updateSensorHum();
  publishValues();

  if (temp_now >= 25 || hum_now >= 80)
  {
    fanspeed = 100;
  }
  else
  {
    fanspeed = 0;
  }
  controlFanSpeed(fanspeed);
}