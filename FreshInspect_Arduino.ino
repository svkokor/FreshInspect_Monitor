#include <Wire.h>
#include <RTClib.h>
#include "DFRobot_BME680_I2C.h"
#include <Multichannel_Gas_GMXXX.h>
#include <SD.h>
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// DEVICES ADRESSES
//#define BME680_I2C_ADDR 0x76 // BME680
#define I2C_ADDRESS 0x3C //DISPLAY
#define RST_PIN -1 //DISPLAY
#define MULTIGAS_I2C 0x08 //MULTIGAS

// INIT DEVICES
RTC_PCF8563 rtc; //CLOCK
DFRobot_BME680_I2C bme(0x76);  //0x77 I2C address
GAS_GMXXX<TwoWire> gas; //MULTIGAS
SSD1306AsciiWire display; //DISPLAY
const int encoderCLK = 2;  // CLK pin of the rotary encoder
const int encoderDT = A1;  // DT pin of the rotary encoder
const int encoderSW = A0;  // SW pin of the rotary encoder
const int chipSelect = 4;  // CS pin of the SD card reader
File myFile;                  // SD card file

// MENU
const int numMenuItems = 3;  // Number of menu items
const char* menuItems[] = {"Record Data", "Live Data", "Time"};
int currentMenuItem = 0;  // Index of the currently selected menu item
int previousMenuItem = -1; // Index of the previously selected menu item
unsigned long selectionStartTime = 1;  // Timestamp of when a menu item was selected

void setup() {
    // SOFT INITS
    Serial.begin(9600);
    Wire.begin();

    // CLOCK INIT
    rtc.begin();

    // MULITGAS INIT
    gas.begin(Wire, MULTIGAS_I2C); // use the hardware I2C

    // ROTARY ENCODER INIT
    pinMode(encoderCLK, INPUT);
    pinMode(encoderDT, INPUT);
    pinMode(encoderSW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderCLK), handleEncoder, CHANGE);

    // SD CARD INIT

if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
}

    // DISPLAY INIT
    #if RST_PIN >= 0
    display.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
    #else // RST_PIN >= 0
    display.begin(&Adafruit128x64, I2C_ADDRESS);
    #endif // RST_PIN >= 0
    display.setFont(Adafruit5x7);

    // BME680 INIT
    uint8_t rslt = 1;
    while(rslt != 0) {
    rslt = bme.begin();
    if(rslt != 0) {
      Serial.println("bme begin failure");
      delay(2000);
    }
  }
  Serial.println("bme begin successful");
  bme.startConvert();
  delay(1000);
  bme.update();

    // DISPLAY WELCOME MESSAGE
    display.clear();
    display.println("Welcome to");
    display.println("SMART-LID");
    display.println("Monitoring System");
    delay(2000);
}

void loop(){
    if (currentMenuItem != previousMenuItem) {
        drawMenu(); // Update menu display only when the current menu item changes
        previousMenuItem = currentMenuItem;
    }

    handleMenuSelection();
}

void handleEncoder() {
  static int previousCLKState = HIGH;
  int currentCLKState = digitalRead(encoderCLK);

  if (previousCLKState == HIGH && currentCLKState == LOW) {
    if (digitalRead(encoderDT) == HIGH) {
      currentMenuItem = (currentMenuItem + 1) % numMenuItems;
    } else {
      currentMenuItem = (currentMenuItem - 1 + numMenuItems) % numMenuItems;
    }
  }

  previousCLKState = currentCLKState;
}

void drawMenu() {
  display.clear();
  display.set2X();
  int yPos = 15;
  for (int i = currentMenuItem - 1; i <= currentMenuItem + 1; i++) {
    if (i >= 0 && i < numMenuItems) {
      display.setCursor(0, yPos);
      if (i == currentMenuItem) {
        display.print(">");
      }
      display.println(menuItems[i]);
      yPos += 8;
    }
  }
}

void handleMenuSelection() {
  if (digitalRead(encoderSW) == LOW) {
    if (currentMenuItem == 1) { // Check if "Live Data" menu item is selected
      
      Serial.println("Live Data Selected");
      showLiveData();
      //delay(1000);
      //collectBME680Data();
      //collectMultiGas();

    } else if (currentMenuItem == 0) { // Check if "Record Data" menu item is selected
      //recordData();
      Serial.println("RECORD Selected");
      delay(1000);
      recordData();
    } else if (currentMenuItem == 2) { // Check if "Time" menu item is selected
      //showTime();
      Serial.println("Time Selected");
      delay(1000);
    }
  }
}

// DATA COLLECTION
float GM102B = 0.0;
float GM302B = 0.0;
float GM502B = 0.0;
float GM702B = 0.0;
float R_102B = 0.0;
float R_302B = 0.0;
float R_502B = 0.0;
float R_702B = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float gas_resistance = 0.0;
float v0=5.00;

void showLiveData(){
  while (digitalRead(encoderSW) == HIGH) {
    delay(10);
    collectMultiGas();
    delay(250);
    float R_102B = 47000 * ((v0 - GM102B) / GM102B);
    float R_302B = 20000 * ((v0 - GM302B) / GM302B);
    float R_502B = 20000 * ((v0 - GM502B) / GM502B);
    float R_702B = 100000 * ((v0 - GM702B) / GM702B);
    bme.startConvert();
    delay(1000);
    bme.update();
    delay(250);
    float temperature = bme.readTemperature() / 100;
    float humidity = bme.readHumidity() / 1000;
    float pressure = bme.readPressure();
    float gas_resistance = bme.readGasResistance();
    display.clear();
    display.set1X();
    display.print("R_102B:");
    display.print(R_102B / 1000.00);
    display.println(" kOhm");
    display.print("R_302B:");
    display.print(R_302B / 1000.00);
    display.println(" kOhm");
    display.print("R_502B:");
    display.print(R_502B / 1000.00);
    display.println(" kOhm");
    display.print("R_702B:");
    display.print(R_702B / 1000.00);
    display.println(" kOhm");
    display.print("Gas:");
    display.print(gas_resistance / 1000.00);
    display.println(" kOhm");
    display.print("Temp:");
    display.print(temperature);
    display.println(" *C");
    display.print("Humidity:");
    display.print(humidity);
    display.println(" %");
}
}

void recordData() {
  
  while (true) {
    display.clear();
    display.set2X();
    display.println("RECORDING");
    delay(60000); // 1-minute delay

    collectMultiGas();
    delay(250);
    float R_102B = 47000 * ((v0 - GM102B) / GM102B);
    float R_302B = 20000 * ((v0 - GM302B) / GM302B);
    float R_502B = 20000 * ((v0 - GM502B) / GM502B);
    float R_702B = 100000 * ((v0 - GM702B) / GM702B);

    bme.startConvert();
    delay(1000);
    bme.update();
    delay(250);

    DateTime now = rtc.now();

    // Save data to SD card by appending to file "DATA.CSV"
    myFile = SD.open("DATA.CSV", FILE_WRITE);
    if (myFile) {
      myFile.print(now.day(), DEC);
      myFile.print(".");
      myFile.print(now.month(), DEC);
      myFile.print(".");
      myFile.print(now.year(), DEC);
      myFile.print(",");
      myFile.print(now.hour(), DEC);
      myFile.print(":");
      myFile.print(now.minute(), DEC);
      myFile.print(":");
      myFile.print(now.second(), DEC);
      myFile.print(",");
      myFile.print(R_102B / 1000.00);
      myFile.print(",");
      myFile.print(R_302B / 1000.00);
      myFile.print(",");
      myFile.print(R_502B / 1000.00);
      myFile.print(",");
      myFile.print(R_702B / 1000.00);
      myFile.print(",");
      myFile.print(bme.readGasResistance() / 1000.00);
      myFile.print(",");
      myFile.print(bme.readTemperature() / 100);
      myFile.print(",");
      myFile.print(bme.readHumidity() / 1000);
      myFile.print(",");
      myFile.print(bme.readPressure() / 100);
      myFile.println();
      myFile.close();
      Serial.println("Data saved to SD card");
    } else {
      Serial.println("Error opening file");
    }
  }
}


void collectMultiGas() {
  uint32_t val = 0;

  // Take measurements from Multichannel Gas sensor
  val = gas.getGM102B();
  GM102B = gas.calcVol(val);
  Serial.println(GM102B);
  delay(500);
  val = gas.getGM302B();
  GM302B = gas.calcVol(val);
  Serial.println(GM302B);
  delay(500);
  val = gas.getGM502B();
  GM502B = gas.calcVol(val);
  Serial.println(GM502B);
  delay(500);
  val = gas.getGM702B();
  GM702B = gas.calcVol(val);
  Serial.println(GM702B);
  delay(500);

  float R_102B = 47000 * ((v0 - GM102B) / GM102B);
  float R_302B = 20000 * ((v0 - GM302B) / GM302B);
  float R_502B = 20000 * ((v0 - GM502B) / GM502B);
  float R_702B = 100000 * ((v0 - GM702B) / GM702B);
}    