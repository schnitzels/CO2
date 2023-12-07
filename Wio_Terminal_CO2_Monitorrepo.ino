#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SensirionI2CScd4x.h>

#include "icon_thermometer.h"
#include "icon_humidity.h"
#include "icon_co2.h"

#define CO2_THRESHOLD 1200

//begin NTP 
//NTP libraries
#include <rpcWiFi.h>
#include <millisDelay.h>
#include "RTC_SAMD51.h"
#include "NTPClient.h"

const char ssid[] = "TP-Link_E2E7"; // add your required ssid
const char password[] = "73723523"; // add your own network password
char ntpServer[] = "0.pool.ntp.org"; // external NTP server e.g. time.nist.gov

// declare a time object
DateTime now;

// define WiFI client
WiFiClient client;

//The udp library class
WiFiUDP udp;

NTPClient timeClient(udp, ntpServer);
//end NTP

SensirionI2CScd4x scd4x;
TFT_eSPI tft = TFT_eSPI();

unsigned int prev_time = 0;


void printUint16Hex(uint16_t value) 
{
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) 
{
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

void setup() {
//NTP setup begin

    Serial.begin(115200);
    while (!Serial);

    connectToWiFi();

//NTP setup end


  Serial.begin(115200);

  pinMode(WIO_5S_PRESS, INPUT_PULLUP);
  pinMode(WIO_BUZZER, OUTPUT);
  pinMode(D0, OUTPUT); // Relay
  digitalWrite(D0, LOW);

  tft.init();
  tft.setRotation(3);
  tft.setTextFont(4);
  tft.fillScreen(TFT_BLACK);

  Wire.begin();
  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    tft.print(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;

  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    tft.print(errorMessage);
  } else {
    printSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    tft.print(errorMessage);
  }

  Serial.println("Waiting for first measurement... (5 sec)");
  tft.setCursor(100, 80);
  tft.print("Waiting...");
  delay(5000);
  tft.fillScreen(TFT_BLACK);
  tft.drawXBitmap(20, 20, icon_co2_bits, icon_co2_width, icon_co2_height, TFT_YELLOW);
  tft.drawXBitmap(20, 90, icon_therm_bits, icon_therm_width, icon_therm_height, TFT_RED);
  tft.drawXBitmap(20, 160, icon_hum_bits, icon_hum_width, icon_hum_height, TFT_CYAN);

}

uint16_t prev_co2 = 0;

void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    while (WiFi.begin(ssid, password) != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nConnected to WiFi");
}

void loop()
{

  unsigned long curr_time = millis();

  if ((curr_time - prev_time) > 5000) {
    prev_time = curr_time;

    uint16_t error;
    char errorMessage[256];

    // Read Measurement
    uint16_t co2;
    float temperature;
    float humidity;

    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      tft.print(errorMessage);
    } 
  timeClient.update();
      Serial.print("Co2:");
      Serial.print(co2);
      Serial.print("\t");
      Serial.print("Temperature:");
      Serial.print(temperature);
      Serial.print("\t");
      Serial.print("Humidity:");
      Serial.println(humidity);
      Serial.print("\t");
      Serial.print("Current time: ");
      Serial.println(timeClient.getFormattedTime());

    delay(1000);  // Adjust the delay as needed


      // clear previous value by setting black font color
      tft.setTextFont(6);
      tft.setCursor(90, 30);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.print(prev_co2);
      tft.setTextFont(4);
      tft.print("ppm");

      tft.setTextFont(6);
      tft.setCursor(90, 30);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.print(co2);
      tft.setTextFont(4);
      tft.print("ppm");
      prev_co2 = co2;

      char temp[5];
      sprintf(temp, "%0.1f", temperature);
      tft.setCursor(90, 100);
      tft.setTextFont(6);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.print(temp);
      tft.setTextFont(4);
      tft.print("`C");

      char hum[5];
      sprintf(hum, "%0.1f", humidity);
      tft.setCursor(90, 170);
      tft.setTextFont(6);
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.print(hum);
      tft.setTextFont(4);
      tft.print("RH");
    
  }}

