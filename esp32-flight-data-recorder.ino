
//amoled include file from https://github.com/VolosR/TDisplayAmoled
#include "rm67162.h"
#include <TFT_eSPI.h>
// display port init
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);


// from https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
#include <Preferences.h>
Preferences FDStorage; // include a data structure for saving flight data (use sparingly)
unsigned int FDStorageFlightCounter;

// load accelerometer
// from https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library/blob/main/examples/example1_basic_readings/example1_basic_readings.ino
#include <Wire.h>
#include <SparkFun_KX13X.h> // Click here to get the library: http://librarymanager/All#SparkFun_KX13X
SparkFun_KX134 kxAccel;
outputData acclEvent; // Struct for the accelerometer's data


// load the library and create the pressure sensor object
// from https://github.com/RobTillaart/MS5611/blob/master/examples/MS5611_minimal_ESP32/MS5611_minimal_ESP32.ino
//#include "MS5611.h"
//MS5611 MS5611(0x77);

const float sea_press = 1016;  // set the sealevel pressure for the altitude calculations

// load pressure sensor
// from https://github.com/adafruit/Adafruit_MPL3115A2_Library/blob/master/examples/testmpl3115a2/testmpl3115a2.ino
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro;



// https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino
#include "FS.h"
#include <LittleFS.h>
/* You only need to format LittleFS the first time you run a test */
#define FORMAT_LITTLEFS_IF_FAILED false


String FDversionNum = "0.6.0";


// set up some font and line size values ... may need to be chaged if not using the amoled lily go esp32: https://github.com/Xinyuan-LilyGO/T-Display-S3-AMOLED
int fontSize = 4;
int textLineSize = 30;
int textLineNum;
int textLineNumStart = 5;

int tog1 = 0; // toggle used to show screen updates

bool tog2 = 0; // toggle used to latch the led button
bool ledON = false;
#define up 21 // up button pin
#define down 0 // down button pin
#define led 38 // led pin
#define s3_sda 43 // amoled t-display pin for i2c sda
#define s3_scl 44 // amoled t-display pin for i2c scl

#define csv_vars         flightNum,millis(),flightCondition,pressGL,altGL,pressure,pressMax,pressMin,temperature,altitude,altMin,altMax,acceleration,acclMin,acclMax
#define csv_description "flightNum,millis(),flightCondition,pressGL,altGL,pressure,pressMax,pressMin,temperature,altitude,altMin,altMax,acceleration,acclMin,acclMax\n"
#define csv_formats "%u,%u,%i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n"


String battery; // string for constructing the output text for the battery voltage
float batteryVoltage; // for storing the battery voltage read from pin 4

unsigned int flightNum = 0;

float altitude;
float altGL = -1;
float pressGL = -1;
float altMin = 9999;
float altMax = -9999;
float altDelta;
float acceleration;
float acclMin = 9999;
float acclMax = -9999;
float temperature;
float tempMin = 9999;
float tempMax = -9999;
float pressure;
float pressMin = 9999;
float pressMax = -9999;

/* flightCondition states:
0 = init
1 = ground hold
2 = ground armed
5 = in flight
8 = landed
*/
String stateMachineBar = "error";
String buttonBar = "-";
String spinner = "|";
int flightCondition = -1 ; 
int stateTransitionTime; // time of last state machine transition

int loopDelay = 100;
unsigned int loopNum = 0;
//int initStateWait     = 1 * 1000;     //how long to pause while going into init state
//int initStateHoldDown = 1 * 1000;     //minimum time required in init state before going to hold state
int holdStateWait     = 1 * 1000;     //how long to pause while going into hold state
//int holdStateHoldDown = 1 * 1000;     //minimum time required hold state before going to arm state
int armStateWait     = 5;              //how long to pause while going into arm state (in seconds)
int armStateHoldDown = 5 * 1000;      //minimum time required arm state before going to flt state
int fltStateHoldDown = 20 * 1000;     //minimum time flight state before going to lnd state

int flightStateAccelTrigger = 3;      //minimum acceleration to trigger flight state
float LandingStateMinAccelTrigger = .95;
float LandingStateMaxAccelTrigger = 1.05;





// littleFS operations from https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    fs::File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    fs::File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}
void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}
void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}
void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    fs::File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}
void writeFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Writing file: %s\r\n", path);

    fs::File file = fs.open(path, FILE_WRITE);
    if(!file){
        //Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        //Serial.println("- file written");
    } else {
        //Serial.println("- write failed");
    }
    file.close();
}
void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\r\n", path);

    fs::File file = fs.open(path, FILE_APPEND);
    if(!file){
        //Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        //Serial.println("- message appended");
    } else {
        //Serial.println("- append failed");
    }
    file.close();
}
void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}
void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}
void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    fs::File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<8; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %lu ms\r\n", 8 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %lu ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}
// end littleFS operations from https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino


float getAltitude(float press, float temp) {
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void setup()
{  
  Serial.begin(115200);
  rm67162_init();  // amoled lcd initialization
  lcd_setRotation(0);
  sprite.createSprite(240, 536);
  sprite.setSwapBytes(1);
  pinMode(up, INPUT_PULLUP); // this is the right button, or top when landscape
  pinMode(down, INPUT_PULLUP); // this is the left button, or bottom when in landscaope
  pinMode(led, OUTPUT); // onboard led

  battery=String(batteryVoltage,1)+"v";

  FDStorage.begin("flight_recorder", false);
  FDStorageFlightCounter = FDStorage.getUInt("flightCounter", 0);
  //Serial.printf("current flightCounter is set to %u\n",FDStorageFlightCounter);
  flightNum = FDStorageFlightCounter + 1;

  sprite.fillSprite(TFT_BLACK);
  sprite.drawString("warming up...",10,10,4); 
  lcd_PushColors(0, 0, 240, 536, (uint16_t*)sprite.getPointer());
  delay(2000);
  setFlightCondition(0); // set condition init
  updateStatusBar();
  draw();
  delay(1000);

  // initalize the i2c bus
  Wire.begin(s3_sda, s3_scl);  //  adjust ESP32 pins if needed

  // bring the barrometer on line
  //while (!Serial);
  //Serial.println(__FILE__);
  //Serial.println("Barometer Test");
  //Serial.print("MS5611_LIB_VERSION: ");
  //Serial.println(MS5611_LIB_VERSION);
  /* ... enable the MS5611
  if (MS5611.begin() == true)
  {
    //Serial.println("MS5611 (barometer) found.");
  }
  else
  {
    Serial.println("MS5611 (barometer) not found. halt.");
    delay(500);
    while (1);
  }
  //Serial.println("");
  */

  if (!baro.begin()) {
    Serial.println("MPL3115A2 (barometer) not found. halt.");
    delay(500);
    while(1);
  }

  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(sea_press);

  /* Initialise the sensor */
  //while (!Serial);
  //Serial.println("Accelerometer Test");
  if (!kxAccel.begin())
  {
    Serial.println("KX134 (accelerometer) not found. halt.");
    delay(500);
    while (1)
      ;
  }
  else {
    //Serial.println("KX134 (accelerometer) found.");
  }
  if (kxAccel.softwareReset())
    //Serial.println("Accel Reset.");

  // Give some time for the accelerometer to reset.
  // It needs two, but give it five for good measure.
  delay(5);

  // Many settings for KX13X can only be
  // applied when the accelerometer is powered down.
  // However there are many that can be changed "on-the-fly"
  // check datasheet for more info, or the comments in the
  // "...regs.h" file which specify which can be changed when.
  kxAccel.enableAccel(false);

  kxAccel.setRange(SFE_KX134_RANGE32G);         // 64g for the KX134

  // kxAccel.enableDataEngine(); // Enables the bit that indicates data is ready.
  kxAccel.setOutputDataRate(50); // Default is 50Hz
  kxAccel.enableAccel();

  //accel.setRange(ADXL343_RANGE_16_G);
  //accel.printSensorDetails();
  //Serial.println("");

  

  updateStatusBar();
  draw();
  //open LittleFS for writing the flight data
  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
        Serial.println("LittleFS Mount Failed");
        return;
    }
}


void setFlightCondition(int newCondition) {
  stateTransitionTime = millis();
  int oldCondition = flightCondition;
  flightCondition = newCondition;

  if (newCondition == 1) { // when ground hold
    if (oldCondition == 0) { // if were moving from init to ground hold, set ground level
      //flightNum++;
      altGL = altitude;
      pressGL = pressure;
    }
    updateStatusBar();
    draw();
    ledON = 1;
    digitalWrite(led, ledON);
    delay(holdStateWait);
  }
  if ((newCondition == 2)&&(oldCondition==1)) { // if were moving from hold to ground arm, do some stuffffff
    updateStatusBar();
    draw();
    int i = 0;
    while(i < armStateWait) {
      ledON = 0;
      digitalWrite(led, ledON);
      delay(500);
      ledON = 1;
      digitalWrite(led, ledON);
      delay(500);
      i++;
    }
    ledON = 0;
    digitalWrite(led, ledON);
    }
  if (newCondition == 5) {  // flight detected
    FDStorage.putUInt("flightCounter", flightNum);  // write our flight number to preferences storage
    FDStorageFlightCounter = FDStorage.getUInt("flightCounter", 0);
    //Serial.printf("in flight ... FD Storage flightCounter is set to %u\n",FDStorageFlightCounter);
    writeFile(LittleFS, "/flight_data.csv", csv_description);
    char buffer[100];
    sprintf(buffer, csv_formats , csv_vars);
    appendFile(LittleFS,  "/flight_data.csv", buffer);
    //appendFile(LittleFS, "/hello.txt", "World!\r\n");
    //readFile(LittleFS, "/hello.txt");
    //Serial.print(csv_description);
    //printCSVtoSerialOutput();
    loopNum = 0;
  }
  if (newCondition == 8) { // we've landed!
    //FDStorage.putUInt("flightCounter", flightNum);  // write our flight number to preferences storage
    //FDStorageFlightCounter = FDStorage.getUInt("flightCounter", 0);
    char buffer[100];
    sprintf(buffer, csv_formats , csv_vars);
    appendFile(LittleFS,  "/flight_data.csv", buffer);
    Serial.printf("landed ... FD Storage flightCounter is set to %u\n",FDStorageFlightCounter);
    //printCSVtoSerialOutput();
  }
}

void printCSVtoSerialOutput () {
  //Serial.printf("%u,%u,%i,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",flightNum,millis(),flightCondition,pressGL,altGL,pressure,pressMax,pressMin,temperature,altitude,altMin,altMax,acceleration,acclMin,acclMax);
  Serial.printf(csv_formats,csv_vars);
}


void advanceSpinner() {
  if (spinner == "|")  {spinner = "/";}
  else if (spinner == "/")  {spinner = "-";}
  else if (spinner == "-")  {spinner = "\\";}
  else {spinner = "|";}
}

void updateStatusBar()
{
  batteryVoltage  = ((analogRead(4) * 2 * 3.3 * 1000) / 4096)/1000;
  battery=String(batteryVoltage,1)+"v";
  advanceSpinner();

  switch (flightCondition) {
    case 0: // init
      stateMachineBar = "f"+String(flightNum)+":INIT alt "+spinner;
      break;
    case 1: // Ground Hold
      stateMachineBar = "f"+String(flightNum)+":hld "+String(altitude,0)+"m "+spinner;
      buttonBar = "arm            hold          clr";
      break;
    case 2: // Ground Armed
      stateMachineBar = "f"+String(flightNum)+":arm "+String(altitude,0)+"m "+spinner;
      buttonBar = "-               hold            -";
      break;
    case 5: // In Flight
      stateMachineBar = "f"+String(flightNum)+":flt "+String(altitude,0)+"m "+spinner;
      break;
    case 8: // Landed
      stateMachineBar = "f"+String(flightNum)+":ldn "+String(altitude,0)+"m "+spinner;
      break;
  }
}


void draw()
{

/*
---------------------|
|f1:hld 1000m -  4.1v|
|GL:1000m    D:3.2m
|min:815m  max:830m
|pressure: 850mb
|min:849   max:851
|accel: 1.03g
|min:1.01g  max:1.05g
|temp: 22.3C
|min:22.3   max:22.4C
|
|
|arm   reset  display|
---------------------|           
*/

  textLineNum = textLineNumStart;
  sprite.fillSprite(TFT_BLACK);
  sprite.drawString(battery,190,textLineNum,4);
  sprite.drawString(stateMachineBar,5,textLineNum,4);
  sprite.drawLine(0,textLineNum+textLineSize,240,textLineNum+textLineSize,TFT_WHITE);

  textLineNum = textLineNum + textLineSize * 1.2;

  sprite.drawString("GL:"+String(altGL,0)+"",5,textLineNum,4);
  sprite.drawString("D:"+String(altDelta,1)+"",120,textLineNum,4);
  textLineNum = textLineNum + textLineSize;
  sprite.drawString("Min:"+String(altMin,0)+"",5,textLineNum,4);
  sprite.drawString("Max:"+String(altMax,0)+"",120,textLineNum,4);
  textLineNum = textLineNum + textLineSize*1.1;

  sprite.drawString("Pressure:"+String(pressure,0)+"mb",5,textLineNum,4);
  textLineNum = textLineNum + textLineSize;
  sprite.drawString("Min:"+String(pressMin,0)+"",5,textLineNum,4);
  sprite.drawString("Max:"+String(pressMax,0)+"",120,textLineNum,4);
  textLineNum = textLineNum + textLineSize*1.1;

  sprite.drawString("Accel:"+String(acceleration,2)+"g",5,textLineNum,4);
  textLineNum = textLineNum + textLineSize;
  sprite.drawString("Min:"+String(acclMin,2)+"",5,textLineNum,4);
  sprite.drawString("Max:"+String(acclMax,2)+"",120,textLineNum,4);
  textLineNum = textLineNum + textLineSize*1.1;

  sprite.drawString("Temp:"+String(temperature,1)+"C",5,textLineNum,4);
  textLineNum = textLineNum + textLineSize;
  sprite.drawString("Min:"+String(tempMin,1)+"",5,textLineNum,4);
  sprite.drawString("Max:"+String(tempMax,1)+"",120,textLineNum,4);
  textLineNum = textLineNum + textLineSize*1.1;

  sprite.drawString("CondTime: "+String((millis() - stateTransitionTime)/1000),5,textLineNum,4);

  sprite.drawString(buttonBar,5,510,4);
  sprite.drawLine(0,508,240,508,TFT_WHITE);
  
  lcd_PushColors(0, 0, 240, 536, (uint16_t*)sprite.getPointer());
}


void readButtons(){
  if ((digitalRead(up) == 0)&&(flightCondition == 1)) { // if right button pressed during hold, reset min/max numbers, and set ground level
    altMin = 9999;
    altMax = -9999;
    altGL = altitude;
    pressGL = pressure;
    pressMin = 9999;
    pressMax = -9999;
    acclMin = 9999;
    acclMax = -9999;
    tempMin = 9999;
    tempMax = -9999;
  }
/*  if (digitalRead(down) == 0) {
    if (tog2 == 0) {
      tog2 = 1;
      ledON = !ledON;
      digitalWrite(led, ledON);
    }
  } else tog2 = 0; */
  if ((digitalRead(down) == 0)&& (digitalRead(up) == 1)) {   // if I press the left button and havn't pressed the right button
    if (flightCondition == 1) {    // and we're in ground hold, advance to armed
      setFlightCondition(2);
    }
  }
  if ((digitalRead(down) == 0) && (digitalRead(up) == 0)) { //reset to ground hold
    stateMachineBar = "10sec hold";
    ledON = 1;
    digitalWrite(led, ledON);
    draw();
    FDStorageFlightCounter = FDStorage.getUInt("flightCounter", 0);
    flightNum = FDStorageFlightCounter + 1;
    Serial.printf("Version: %s\n",FDversionNum);
    Serial.printf("flight reset ... FD Storage flightCounter is set to %u\n",FDStorageFlightCounter);
    listDir(LittleFS, "/", 3);
    readFile(LittleFS,  "/flight_data.csv");
    delay(10000);
    setFlightCondition(1); // move to ground hold
  }
}


void getAccel() {
    /* Get a new sensor event */
  //sensors_event_t event;
  //accel.getEvent(&event);
  kxAccel.getAccelData(&acclEvent);

  // This particular kx143 appears to need a slight adjustment to calibrate
  acclEvent.zData = acclEvent.zData - .1;
  acclEvent.yData = acclEvent.yData - .05;

  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("X: "); Serial.print(acclEvent.xData); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(acclEvent.yData); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(acclEvent.zData); Serial.print("  ");Serial.println("g ");
  //Serial.print("G: "); 
  acceleration = sqrt(pow(acclEvent.xData,2) + pow(acclEvent.yData,2) + pow(acclEvent.zData,2));
  //Serial.println( acceleration );

  if (acclMin > acceleration) {acclMin = acceleration;} 
  if (acclMax < acceleration) {acclMax = acceleration;}
}

void getBaro() {
  //MS5611.read();           //  note no error checking => "optimistic".
  //Serial.print("T(C):\t");
  //Serial.print(MS5611.getTemperature(), 2);
  //Serial.print("\tP(mb):\t");
  //Serial.print(MS5611.getPressure(), 2);
  //Serial.print("\tA(m):\t");
  //Serial.print(getAltitude(MS5611.getPressure(),MS5611.getTemperature()) , 2);
  //Serial.println();


  /* ... old way for MS5611
  temperature = MS5611.getTemperature();  
  pressure = MS5611.getPressure();
  altitude = getAltitude(pressure,temperature);
  */

  pressure = baro.getPressure();
  altitude = baro.getAltitude();
  temperature = baro.getTemperature();

  if (tempMin > temperature) {tempMin = temperature;} 
  if (tempMax < temperature) {tempMax = temperature;}

  if (pressMin > pressure) {pressMin = pressure;} 
  if (pressMax < pressure) {pressMax = pressure;}

  if (altMin > altitude) {altMin = altitude;} 
  if (altMax < altitude) {altMax = altitude;}
  altDelta = (altitude - altGL);

}


void loop()
{
  readButtons();
  //batteryVoltage  = ((analogRead(4) * 2 * 3.3 * 1000) / 4096)/1000;
  //battery=String(batteryVoltage,1)+"v";

  getAccel();
  getBaro();


  switch (flightCondition) {
    case 0: // init
      setFlightCondition(1); // move from init to ground hold
      break;
    case 1: // Ground Hold
      
      break;
    case 2: // Ground Armed
      if ( ((millis() - stateTransitionTime) > armStateHoldDown) && 
        (acceleration > flightStateAccelTrigger) ) {
          setFlightCondition(5);
          }
      break;
    case 5: // In Flight
      if (!(loopNum % 5)) {
        //FDStorage.putUInt("flightCounter", flightNum);  // write our flight number to preferences storage
        //FDStorageFlightCounter = FDStorage.getUInt("flightCounter", 0);
        char buffer[200];
        sprintf(buffer, csv_formats , csv_vars);
        appendFile(LittleFS,  "/flight_data.csv", buffer);
        //printCSVtoSerialOutput();
        //Serial.printf("in flight ... loopNum: %u\n",loopNum);
      }
      if ( ((millis() - stateTransitionTime) > fltStateHoldDown) && 
        (acceleration > LandingStateMinAccelTrigger) &&
        (acceleration < LandingStateMaxAccelTrigger) ) {
          setFlightCondition(8);
      }
      loopNum++;
      break;
    case 8: // Landed
      
      break;
  }

  updateStatusBar();
  draw();
  delay(loopDelay);

}