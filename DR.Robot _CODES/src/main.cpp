//******************************************ADD YOUR #INCLUDE HERE*************************************************************************
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <DHT.h>
#include "time.h"
#include <Wire.h>
#include <WiFi.h>
#include <string.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <ESP32_FTPClient.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm
//*****************************************ADD YOU #DEFINE HERE*****************************************************************************
#define DHT_SENSOR_PIN 13     // ESP32 pin GIOP39 connected to DHT22 sensor.
#define DHT_SENSOR_TYPE DHT11 // DHT sensor type is DHT22.
//*****************************************DEFINE YOUR STRUCTURE HERE************************************************************************

//*****************************************ADD YOUR CONFIGURATIONS VARIABLES HERE*********************************************************
int lcdColumns = 20;
int lcdRows = 4;
bool SD_OK = 0;
// int debug = 1;
bool pcdef = 1;
bool pc_1 = 1;
int page_no = 0;
const int gmtOffset_sec = 19800; // time zone * 3600 , my time zone is  + 05:30 GTM (so 5.5X3600 =19800).
const int daylightOffset_sec = 19800;
const char *ssid = "Hare Krishna-4G";
const char *password = "@VandeMatram";
const char *ntpServer = "pool.ntp.org";

//*****************************************DEFINE YOUR GLOBAL VARIABLES HERE*****************************************************************

long irValue = 0;
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
// File file = SD.open("/datalog_ESP32.txt", FILE_READ);
MAX30105 particleSensor;
const byte RATE_SIZE = 8; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

// static const unsigned char PROGMEM logo3_bmp[] =
//     {0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
//      0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
//      0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
//      0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
//      0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
//      0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
//      0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
//      0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00};
struct tm timeinfo;
LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);
//*****************************************DECLARE YOUR FUNCTIONS HERE*****************************************************************************

void initWiFi();
void cardType();
void drawKeypad();
void getLocalTime();
void printLocalTime();
void touch_calibrate();
unsigned long getTime();
void status(const char *msg);
void KeyInput(void *parameter);
void SendData(void *parameter);
void SaveData(void *parameter);
void AllSensorsData(void *param);
void DisplayScreen(void *parameter);
void readFile(fs::FS &fs, const char *path);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);

//****************************************PROGRAM STARTS FROM HERE***************************************************************************

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);  // LED indication.
  pinMode(15, OUTPUT); // buzzer output
  lcd.init();          // initialize LCD
  lcd.backlight();     // turn on LCD backlight
  lcd.setCursor(0, 1);
  digitalWrite(2, HIGH);
  digitalWrite(15, HIGH);
  lcd.print("Starting...");
  delay(1000);
  digitalWrite(2, LOW);
  digitalWrite(15, LOW);
  lcd.clear();
  dht_sensor.begin();                                       // initialize the DHT sensor
  initWiFi();                                               // initializing Wi-Fi.
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // configuring time from NTP server.
  Serial.println("Getting date & time  ");
  lcd.setCursor(0, 1);
  lcd.print("Getting date & time ");
  delay(5000);
  Serial.print("CURRENT DATE & TIME IS : ");
  getLocalTime(&timeinfo);
  printLocalTime();
  setTime(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
  Serial.println("Date & time updated.");
  lcd.print("Date & time updated.");

  // if (!SD.begin(5)) // if SD card not begin.
  // {
  //   SD.end();
  //   if (!SD.begin(5)) // if SD card not begin.
  //   {
  //     SD_OK = 0;
  //     Serial.println("SD Card Not Found !");
  //     tft.setTextColor(TFT_RED, TFT_BLUE);
  //     tft.drawString("SD card not found !", 5, 180, 2);
  //     tft.drawString("SD card insert fail ! Please insert SD card & reset the device.", 5, 195, 2);
  //     // return;
  //   }
  // }

  // // if SD card begin.
  // if (SD.begin(5))
  // {
  //   SD_OK = 1;
  //   Serial.println("SD Card > OK.");
  //   lcd.print("SD card > OK.");
  //   Serial.println("SD Card Details: ");
  //   Serial.print("SD Card Type: ");
  //   cardType();
  //   writeFile(SD, "/DeviceLog.txt", "SD card insert OK.\n");
  //   listDir(SD, "/", 0);
  // }

  // particleSensor.begin(Wire, I2C_SPEED_FAST); // Use default I2C port, 400kHz speed
  // particleSensor.setup();                     // Configure sensor with default settings
  // delay(2000);

  // xTaskCreate(AllSensorsData, "AllSensorsData", 8000, NULL, 1, NULL);
  // //    xTaskCreate(SaveData, "SaveData", 8000, NULL, 1, NULL);
  // xTaskCreate(DisplayScreen, "DisplayScreen", 8000, NULL, 1, NULL);
  //  xTaskCreate(KeyInput, "KeyInput", 8000, NULL, 1, NULL);
  //  xTaskCreate(SendData, "SendData", 8000, NULL, 2, NULL);
  delay(1000);
  lcd.clear();
}

void loop() // void loop function.
{
  lcd.setCursor(0, 0);
  lcd.print(String(day()) + "/" + String(month()) + "/" + String(year()) + "   ");
  lcd.print(String(hour()) + ":" + String(minute()) + ":" + String(second()));
  float humi = dht_sensor.readHumidity();
  float temp = dht_sensor.readTemperature();
  // set cursor to first column, first row
  lcd.setCursor(0, 2);
  lcd.print("Temperature: ");
  lcd.print(temp);
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("ºC ");
  lcd.setCursor(0, 3);
  lcd.print("Humidity: ");
  lcd.print(humi);
  Serial.print("Humidity: ");
  Serial.println(humi);
  irValue = particleSensor.getIR(); // Reading the IR value it will permit us to know if there's a finger on the sensor or not Also detecting a heartbeat.
  if (irValue > 7000)               // If a finger is detected.
  {
    // particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    if (checkForBeat(irValue) == true) // If a heart beat is detected.
    {
      Serial.print(" BPM  :");
      Serial.println(beatAvg);
      lcd.setCursor(0,1);
      lcd.print(beatAvg);
      long delta = millis() - lastBeat; // Measure duration between two beats
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0); // Calculating the BPM

      if (beatsPerMinute < 255 && beatsPerMinute > 20) // To calculate the average we strore some values (4) then do some math to calculate the average
      {
        rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE;                    // Wrap variable
        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
      // tft.drawBitmap(300, 240, logo3_bmp, 32, 32, TFT_BLUE);
    }
  }
  if (irValue < 7000) // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
  {
    // tft.drawString("TUCH THE SENSOR FOR READING.   ", 200, 200, 2);
    // tft.drawBitmap(300, 240, logo3_bmp, 32, 32, TFT_BLUE);
    // tft.drawString("BPM: 0  ", 320, 150, 4);
    delay(1000);
    //   // lcd.clear();
  }
}
  /*
  //------------------------------------------------------------------------------------------
  void AllSensorsData(void *param)
  {
    for (;;)
    {
      irValue = particleSensor.getIR(); // Reading the IR value it will permit us to know if there's a finger on the sensor or not Also detecting a heartbeat.
      if (irValue > 7000)               // If a finger is detected.
      {
        page_no = 1;
        // particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
        if (checkForBeat(irValue) == true) // If a heart beat is detected.
        {
          // Serial.print(" BPM  :");
          // Serial.println(beatAvg);
          long delta = millis() - lastBeat; // Measure duration between two beats
          lastBeat = millis();
          beatsPerMinute = 60 / (delta / 1000.0); // Calculating the BPM

          if (beatsPerMinute < 255 && beatsPerMinute > 20) // To calculate the average we strore some values (4) then do some math to calculate the average
          {
            rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
            rateSpot %= RATE_SIZE;                    // Wrap variable
            // Take average of readings
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
              beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
          }
          // tft.drawBitmap(300, 240, logo3_bmp, 32, 32, TFT_BLUE);
        }
      }
      if (irValue < 7000) // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
      {
        beatAvg = 0;
        page_no = 0;
        // tft.drawString("TUCH THE SENSOR FOR READING.   ", 200, 200, 2);
        // tft.drawBitmap(300, 240, logo3_bmp, 32, 32, TFT_BLUE);
        // tft.drawString("BPM: 0  ", 320, 150, 4);
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }

  void DisplayScreen(void *parameter) // Function for Reading voltage value from analog chanel and write it into SD card.
  {
    for (;;)
    {
      switch (page_no)
      {
      case 0:
        if (pcdef)
        {
          // set the TFT backgroung color for default screen.
          pc_1 = 1;
          pcdef = 0;
        }

        tft.drawString(String(day()) + " . " + String(month()) + " . " + String(year()));

        tft.drawString(String(hour()) + " : " + String(minute()) + " : " + String(second()));
        break;

      case 1:
        // Draw the fix components of defalt screen on LCD.
        if (pc_1)
        {
          tft.fillScreen(TFT_BLACK);                // set the TFT backgroung color for screen 1.
          tft.drawRect(0, 30, 480, 290, TFT_BLUE);  // Draw the Big Frame Blue Rectangle.
          tft.drawRect(10, 40, 220, 120, TFT_RED);  // Draw the Envirment info Box.
          tft.drawRect(10, 170, 220, 90, TFT_RED);  // Draw the System info Box.
          tft.drawRect(240, 40, 230, 220, TFT_RED); // Draw USER info Box.
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.drawString("[MENU] ", 10, 5, 4);
          tft.drawString(" ENVIORNMENT ", 70, 50, 2);
          tft.drawString(" SYSTEM ", 90, 180, 2);
          tft.drawString(" USER ", 330, 50, 2);
          tft.setTextColor(TFT_CYAN, TFT_BLACK);
          tft.drawString("TEMPERATURE : ", 20, 80, 2);
          tft.drawString("HUMIDITY      : ", 20, 100, 2);
          tft.drawString("AIR QUALITY   : ", 20, 120, 2);
          tft.drawString("TEMPERATURE : ", 20, 200, 2);
          tft.drawString("WI-FI MAX30102  SD  RTC  DHT", 20, 220, 2);
          tft.drawString("NAME : AMRIT", 260, 80, 2);
          tft.drawString("MOB. NO. : 9889051334", 260, 100, 2);
          tft.drawString("BPM : ", 260, 160, 4);
          tft.drawString("SPO2 : ", 260, 190, 4);
          tft.drawString("TEMP. : ", 260, 220, 4);
          tft.drawBitmap(410, 160, logo3_bmp, 32, 32, TFT_RED);
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
          tft.drawString(" KEEP YOUR FINGER STURDY", 260, 130, 2);
          tft.drawString("CON.     OK       OK  OK    OK", 20, 240, 2);
          tft.drawString("PROJECT NAME: HEALTH CARE DEVICE", 10, 280, 4);
          pcdef = 1;
          pc_1 = 0;
        }
        // draw dynemic componets of defalt screen on TFT.
        if (isnan(dht_sensor.readTemperature()) || isnan(dht_sensor.readTemperature(true)) || isnan(dht_sensor.readHumidity()))
        {
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
          tft.drawString("26.50 C  ", 130, 80, 2);
          tft.drawString("27.60 %  ", 130, 100, 2);
          tft.drawString("GOOD  ", 130, 120, 2);
        }
        else
        {
          // tft.setTextColor(TFT_WHITE, TFT_BLACK);
          tft.drawString(String(dht_sensor.readTemperature()) + " C", 130, 80, 2);
          // tft.drawString(String(dht_sensor.readTemperature(true)) + "F", 130, 80, 2);
          tft.drawString(String(dht_sensor.readHumidity()) + " %", 130, 100, 2);
          tft.drawString("GOOD  ", 130, 120, 2);
        }
        vTaskDelay(600 / portTICK_PERIOD_MS);
        tft.drawString(String(beatAvg), 340, 160, 4);
        digitalWrite(2, HIGH);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        digitalWrite(2, LOW);
        tft.drawString(" 98 %", 340, 190, 4);
        tft.drawString(" 36 C", 350, 220, 4);
        tft.drawString(String(day()) + " . " + String(month()) + " . " + String(year()), 340, 5, 4);
        tft.drawString(String(hour()) + " : " + String(minute()), 200, 5, 4);
        tft.drawString(String(rtc.getTemperature()) + " C", 130, 200, 2);
        break;

      case 2:
        // tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_BLACK, TFT_RED);
        tft.drawString("page_no: " + String(page_no), 0, 0, 4);
        break;

      case 3:
        // tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_BLACK, TFT_RED);
        tft.drawString("page_no: " + String(page_no), 0, 0, 4);
        break;

      case 4:
        // tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_BLACK, TFT_RED);
        tft.drawString("page_no: " + String(page_no), 0, 0, 4);
        break;

        // default:
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  void SaveData(void *parameter)
  {
    for (;;)
    {
      // Serial.println("SaveData");
      //  tft.fillScreen(TFT_BLACK);
      //  tft.setTextColor(TFT_BLACK, TFT_RED);
      //  tft.drawString("  SaveData  ", 0, 0, 4);
      if (1)
      {
        writeFile(SD, "File name", "Date & Time,Data\n");
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  void SendData(void *parameter)
  {
    for (;;)
    {
      // Serial.println("SendData");
      //  tft.fillScreen(TFT_BLACK);
      //  tft.setTextColor(TFT_BLACK, TFT_RED);
      //  tft.drawString("  SendData  ", 0, 0, 4);

      // listDir(SD, "/", 0);
      // else if (!digitalRead(DI_1))
      // {
      //   selectFileToUploade(SD, "/", 0);
      // }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  void KeyInput(void *parameter)
  {
    for (;;)
    {
      // Serial.println("KeyInput");
      //  tft.fillScreen(TFT_BLACK);
      //  tft.setTextColor(TFT_BLACK, TFT_RED);
      //  tft.drawString("  KeyInput  ", 0, 0, 4);
      //  vTaskDelay(1000 / portTICK_PERIOD_MS);
      // Serial.println(Serial2.read());
    }
  }
  */
  //***********************************************************************************************************************************************************

  void initWiFi() // Function for initializing Wi-Fi.
  {
    WiFi.mode(WIFI_STA); // setting Wi-Fi_MODE.
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++)
    {
      Serial.println(WiFi.SSID(i));
      Serial.println(WiFi.RSSI(i));
    }
    WiFi.begin(ssid, password); // Starting Wi-Fi.
    Serial.print("Connecting to: ");
    Serial.print(ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print('.');
      delay(1000);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Connected to Wi-Fi");
      Serial.print("Wi-Fi Local IP - ");
      Serial.println(WiFi.localIP());
    }
  }

  void cardType()
  {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_MMC)
    {
      Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
      Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
      Serial.println("SDHC");
    }
    else
    {
      Serial.println("UNKNOWN");
    }
  }

  unsigned long getTime() // Function that gets current epoch time.
  {
    time_t now;
    struct tm timeinfo;

    if (!getLocalTime(&timeinfo))
    {
      Serial.println("Failed to Obtain Time");
      return (0);
    }
    time(&now);
    return now;
    Serial.println("Time Obtained Sucsessfully");
  }

  void printLocalTime()
  {
    if (!getLocalTime(&timeinfo))
    {
      Serial.println("Failed to obtain time");
      return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }

  void openDir(fs::FS & fs, const char *dirname, uint8_t levels)
  {
    Serial.printf("Directory open: %s\n", dirname);

    File root = fs.open(dirname);
  }

  void createDir(fs::FS & fs, const char *path)
  {
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
      Serial.println("Dir created");
    }
    else
    {
      Serial.println("mkdir failed");
    }
  }

  void removeDir(fs::FS & fs, const char *path)
  {
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path))
    {
      Serial.println("Dir removed");
    }
    else
    {
      Serial.println("rmdir failed");
    }
  }

  void readFile(fs::FS & fs, const char *path)
  {
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file)
    {
      Serial.println("Failed to open file for reading");
      return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
      Serial.write(file.read());
    }
    file.close();
  }

  void renameFile(fs::FS & fs, const char *path1, const char *path2)
  {
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2))
    {
      Serial.println("File renamed");
    }
    else
    {
      Serial.println("Rename failed");
    }
  }

  void deleteFile(fs::FS & fs, const char *path)
  {
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path))
    {
      Serial.println("File deleted");
    }
    else
    {
      Serial.println("Delete failed");
    }
  }

  void testFileIO(fs::FS & fs, const char *path)
  {
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if (file)
    {
      len = file.size();
      size_t flen = len;
      start = millis();
      while (len)
      {
        size_t toRead = len;
        if (toRead > 512)
        {
          toRead = 512;
        }
        file.read(buf, toRead);
        len -= toRead;
      }
      end = millis() - start;
      Serial.printf("%u bytes read for %u ms\n", flen, end);
      file.close();
    }
    else
    {
      Serial.println("Failed to open file for reading");
    }

    file = fs.open(path, FILE_WRITE);
    if (!file)
    {
      Serial.println("Failed to open file for writing");
      return;
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++)
    {
      file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
  }

  void writeFile(fs::FS & fs, const char *path, const char *message) // Function for Creating new file on SD card.
  {
    Serial.printf("Creating new file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
      Serial.println("Failed to Open File For Writing");
      return;
    }
    if (file.print(message))
    {
      Serial.println("File has been created");
    }
    else
    {
      Serial.println("Write Failed");
    }
    file.close();
  }

  void appendFile(fs::FS & fs, const char *path, const char *message) // Function for Appending data on SD card.
  {
    File file = fs.open(path, FILE_APPEND); // Serial.printf("Appending to file: %s\n", path);
    if (!file)
    {
      Serial.println("Failed to open file for appending");
      return;
    }
    if (file.print(message))
    {
      Serial.println("Message appended");
    }
    else
    {
      Serial.println("Append Failed");
    }
    file.close();
  }

  void listDir(fs::FS & fs, const char *dirname, uint8_t levels)
  {
    Serial.printf("Listing directory: %s\n", dirname);
    File root = fs.open(dirname);
    if (!root)
    {
      Serial.println("Failed to open directory");
      return;
    }
    if (!root.isDirectory())
    {
      Serial.println("Not a directory");
      return;
    }

    File file = root.openNextFile();
    while (file)
    {
      if (file.isDirectory())
      {
        Serial.print("DIR : ");
        Serial.println(file.name());
        time_t t = file.getLastWrite();
        struct tm *tmstruct = localtime(&t);
        // Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", timeinfo.tm_mday, (timeinfo.tm_mon) + 1, (timeinfo.tm_year) + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        Serial.print("  SIZE: ");
        Serial.print(file.size());
        Serial.println("KB");
        if (levels)
        {
          listDir(fs, file.path(), levels - 1);
        }
      }
      else
      {
        Serial.print("FILE: ");
        Serial.println(file.name());
        time_t t = file.getLastWrite();
        struct tm *tmstruct = localtime(&t);
        // Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", timeinfo.tm_mday, (timeinfo.tm_mon) + 1, (timeinfo.tm_year) + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        Serial.print("  SIZE: ");
        Serial.print(file.size());
        Serial.println("KB");
      }
      file = root.openNextFile();
    }
  }