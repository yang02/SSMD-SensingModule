/*LIBRARIES ----------*/
#include <SD.h>
#include <LiquidCrystal.h>
#include <dht.h>

/*PIN ASSIGN ----------*/
#define MIC    A0
#define DHT22  6 //Temperture & Humidity
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2
#define LCD_RS 7
#define LCD_RW 9
#define LCD_EN 10
// Sparkfun's microSD shield uses 8(for CS), 11, 12, 13
#define SD_CS  8

dht DHT;
LiquidCrystal lcd = LiquidCrystal(7,9,10,5,4,3,2);
File dataFile;


boolean serialCom = false;
boolean sdWrite = true;

//basic const prams
const int SENSOR_NUM = 3;
const int BUFFER_LENGTH = 10;
const int SAMPLE_WIN = 50; // Sample window width in mS (50 mS = 20Hz)

//signal ranges
const int SIG_MIN = 0;
const int SIG_MAX = 1024;
const int TEMP_MIN = -20;
const int TEMP_MAX = 50;
const int HUM_MIN = 0;
const int HUM_MAX = 100;

//get average
const int duration = 1000;
unsigned long timestamp;
float buffer[SENSOR_NUM];
float val[SENSOR_NUM];
int index = 0;
float preTemp = 0;
float preHum = 0;

byte inByte = 0;  // incoming serial byte

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // SETUP SD CARD ////////////////////
  Serial.print(F("Initializing SD card..."));
  pinMode(10, OUTPUT);
  if (!SD.begin(SD_CS)) {
    Serial.println(F("Card failed, or not present"));
    while(1);
  }
  Serial.println(F("ok."));
  SdFile::dateTimeCallback( &dateTime ); //register date for file
  dataFile = SD.open("testLog.txt", FILE_WRITE);
  dataFile.seek(0);

  // SETUP LCD ///////////////////////
  lcd.begin(16, 2);
  lcd.clear();
  lcd.noBlink();

  // send a byte to establish contact until receiver responds 
  if(serialCom) establishContact();

  timestamp = 0;
}

void loop()
{


  int chk = DHT.read22(DHT22);

  buffer[0] += getVolume(MIC);
  if(chk == DHTLIB_OK){
    buffer[1] += DHT.temperature;
    buffer[2] += DHT.humidity;
    preTemp = buffer[1];
    preHum = buffer[2];
  }
  else{
    buffer[1] += preTemp;
    buffer[2] += preHum;
  }
  index++;

  //  for(int i=0; i<SENSOR_NUM; i++){
  //    buffer[i][index] = int(data[i]);
  //    val[i] = getSmoothedData(buffer[i]);
  //    index = (index + 1) % BUFFER_LENGTH;
  //  }

  if(timestamp + duration < millis()) {

    for(int i=0; i<SENSOR_NUM; i++){
      val[i] = buffer[i] / index;
    }

    // if we get a valid byte, read analog ins:
    if (Serial.available() > 0 && serialCom) {
      // get incoming byte:
      inByte = Serial.read();

      for(int i=0; i<SENSOR_NUM; i++){
        int mappedVal;
        if(i == 1) mappedVal = map(int(val[1]),TEMP_MIN,TEMP_MAX,SIG_MIN,SIG_MAX);
        else if(i == 2) mappedVal = map(int(val[2]),HUM_MIN,HUM_MAX,SIG_MIN,SIG_MAX);
        else mappedVal = int(val[0]);

        Serial.write(mappedVal >> 8);
        Serial.write(mappedVal & 255);
        delay(5);
      }
    }
    else{
      Serial.print(getTime(millis()));
      Serial.print("\t");
      Serial.print(val[0]);
      Serial.print("\t");
      Serial.print(val[1]);
      Serial.print("\t");
      Serial.print(val[2]);
      Serial.print("\t");
      Serial.print(index);
      Serial.println();

      dataFile = SD.open("testLog.txt", FILE_WRITE);
      if(dataFile){
        dataFile.print(getTime(millis()));
        dataFile.print("\t");
        dataFile.print(val[0]);
        dataFile.print("\t");
        dataFile.print(val[1]);
        dataFile.print("\t");
        dataFile.print(val[2]);
        dataFile.println();
        dataFile.close();
      }
      else {
        Serial.println(F("error opening datalog.txt"));
      }
    }

    for(int i=0; i<SENSOR_NUM; i++){
      buffer[i] = 0;
    }
    timestamp = millis();
    index = 0;
  }
}

/* SERIAL CONNECTION --------------------------------*/
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}

/* MEAN FILITER ------------------------------------- */
int getSmoothedData(int b[]) {
  long sum = 0;
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    sum += b[i];
  }
  return (int)(sum / BUFFER_LENGTH);
}

/* GET VOLUME */
int getVolume(int p){
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int sample;

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  while (millis() - startMillis < SAMPLE_WIN){
    sample = analogRead(p);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax){
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin){
        signalMin = sample;  // save just the min levels
      }
    }
  }
  return signalMax - signalMin;  // max - min = peak-peak amplitude  
}


/* GET DATE --------------------------------------- */
void dateTime(uint16_t* date, uint16_t* time)
{
  uint16_t year = 2014;
  uint8_t month = 4, day = 19, hour = 9, minute = 0, second = 0;

  *date = FAT_DATE(year, month, day);
  *time = FAT_TIME(hour, minute, second);
}

String getTime(uint32_t mls){
  String h = String(mls / 3600000);
  if(mls / 3600000 < 10) h = "0" + h;
  String m = String(mls / 60000);
  if(mls / 60000 < 10) m = "0" + m;  
  String s = String((mls / 1000) % 60);
  if((mls / 1000) % 60 < 10) s = "0" + s;
  return h + "." + m + "." + s;
}





