/*LIBRARIES ----------*/
#include <SD.h>
#include <LiquidCrystal.h>
#include <dht.h>


/*PIN ASSIGN ----------*/
#define MIC    A0
#define POT    A4 // set the interval of record data
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
#define SWITCH 19 // < not fixed yet

//distance sensor starts from PIN 22
#define DIST_PIN_OFFSET 22
/*PIN ASSIGN ----------*/


/*INIT LIBS OBJECTS ----------*/
dht DHT;
LiquidCrystal lcd = LiquidCrystal(7,9,10,5,4,3,2);
File dataFile;

boolean serialCom;
char filename[] = "LOG000.TXT";

//basic const prams
const int SENSOR_NUM_ENV = 3;
const int SENSOR_NUM_DIST = 3;
const int SENSOR_NUM = SENSOR_NUM_ENV + SENSOR_NUM_DIST;
const int SAMPLE_WIN = 50; // Sample window width in mS (50 mS = 20Hz)
const int BUFFER_LENGTH = 5;

//signal ranges
const int SIG_MIN = 0;
const int SIG_MAX = 1024;
const int TEMP_MIN = -20;
const int TEMP_MAX = 50;
const int HUM_MIN = 0;
const int HUM_MAX = 100;
const int DIST_MIN = 0;
const int DIST_MAX = 400;

unsigned int interval;
unsigned long timestamp;

//get average
int buffer[SENSOR_NUM][BUFFER_LENGTH];
int index = 0;
float preTemp = 0;
float preHum = 0;

int val[SENSOR_NUM];
byte inByte = 0;  // incoming serial byte

//for getDistance function
int duration[SENSOR_NUM];


void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  //initialize distance sensor pins
  for(int i=0; i<SENSOR_NUM_DIST; i++){
    pinMode(DIST_PIN_OFFSET+(i*2), OUTPUT);
    pinMode(DIST_PIN_OFFSET+(i*2+1), INPUT);
  }
  
  // serialCom = digitalRead(SWITCH);
  //  interval = map(analogRead(4),0,1023,1000,20000);
  //  interval = int(interval / 1000) * 1000;

  // SETUP SD CARD ////////////////////
  Serial.print(F("Initializing SD card..."));
  pinMode(10, OUTPUT);
  if (!SD.begin(SD_CS)) {
    Serial.println(F("Card failed, or not present"));
    //      while(1);
  }
  Serial.println(F("ok."));
  SdFile::dateTimeCallback( &dateTime ); //register date for

  //set the file name
  for(uint8_t i=0; i<1000; i++){
    filename[4] = i/100 + '0';
    filename[4] = i/10 + '0';
    filename[5] = i%10 + '0';
    if(!SD.exists(filename)){
      dataFile = SD.open(filename, FILE_WRITE);
      break;
    }
  }
  dataFile.seek(0);


  // SETUP LCD ///////////////////////
//  lcd.begin(16, 2);
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print(filename);    
//  lcd.setCursor(0,1);
//  lcd.print("INTERVAL:");    
//  lcd.setCursor(9,1);
//  lcd.print(interval/1000);
//  lcd.print("s");
//  lcd.noBlink();
//  delay(4000);    


  //send a byte to establish contact until receiver responds 
  establishContact();

  timestamp = 0;
}

void loop()
{
  //
  float data[SENSOR_NUM];
  
  int chk = DHT.read22(DHT22);

  //set mic vol
  data[0] = getVolume(MIC);

  //set temp & humidity
  if(chk == DHTLIB_OK){
    data[1] = DHT.temperature;
    data[2] = DHT.humidity;
    preTemp = data[1];
    preHum = data[2];
  }
  else{
    data[1] = preTemp;
    data[2] = preHum;
  }

  //get distances
  for(int i=0; i<SENSOR_NUM_DIST; i++){
    int n = i + SENSOR_NUM_ENV;
    data[n] = getDistance(DIST_PIN_OFFSET+(i*2), DIST_PIN_OFFSET+(i*2+1), i);
  }

  //smoothing data
  for(int i=0; i<SENSOR_NUM; i++){
    buffer[i][index] = data[i];
    val[i] = getSmoothedData(buffer[i]);
  }
  index = (index + 1) % BUFFER_LENGTH;

  //Serial communication
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0 && serialCom) {
    // get incoming byte:
    inByte = Serial.read();

    for(int i=0; i<SENSOR_NUM; i++){
      int mappedVal;
      if(i == 0) mappedVal = int(val[0]);
      if(i == 1) mappedVal = map(int(val[1]),TEMP_MIN,TEMP_MAX,SIG_MIN,SIG_MAX);
      if(i == 2) mappedVal = map(int(val[2]),HUM_MIN,HUM_MAX,SIG_MIN,SIG_MAX);
      if(2 < i)  mappedVal = map(int(val[i]),DIST_MIN, DIST_MAX,SIG_MIN,SIG_MAX);

      Serial.write(mappedVal >> 8);
      Serial.write(mappedVal & 255);
      delay(5);
    }
  }

  //Prints
//  else{
//    // display serial monitor //////////
//    for(int i=0; i<SENSOR_NUM; i++){    
//      Serial.print(val[i]);
//      Serial.print("\t");
//    }
//    Serial.println();

//    // display LCD //////////
//    lcd.clear();
//    lcd.setCursor(0,0);
//    lcd.print("TIME: " + getTime(millis()));
//    lcd.setCursor(0,1);
//    lcd.print("V:");
//    lcd.print(val[0]);
//    lcd.setCursor(6,1);
//    lcd.print("T:");
//    lcd.print(val[1]);
//    lcd.setCursor(11,1);
//    lcd.print("H:");
//    lcd.print(val[2]);
//
//    // write microSD card //////////
//    dataFile = SD.open(filename, FILE_WRITE);
//    if(dataFile){
//      dataFile.print(getTime(millis()));
//      dataFile.print("\t");
//      dataFile.print(val[0]);
//      dataFile.print("\t");
//      dataFile.print(val[1]);
//      dataFile.print("\t");
//      dataFile.print(val[2]);
//      dataFile.println();
//      dataFile.close();
//    }
//    else {
//      Serial.println(F("error opening datalog.txt"));
//    }
//  }
}


//////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////

/* SERIAL CONNECTION --------------------------------*/
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}
/* MEAN FILITER -------------------------------------*/
int getSmoothedData(int b[]) {
  long sum = 0;
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    sum += b[i];
  }
  return (int)(sum / BUFFER_LENGTH);
}
/* GET VOLUME ---------------------------------------*/
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
/* GET DISTANCE ---------------------------------------*/
int getDistance(int trig, int echo, int i){
  /* The following trigPin/echoPin cycle is used to determine the
   distance of the nearest object by bouncing soundwaves off of it. */
  digitalWrite(trig, LOW); 
  delayMicroseconds(2); 

  digitalWrite(trig, HIGH);
  delayMicroseconds(10); 

  digitalWrite(trig, LOW);
  duration[i] = pulseIn(echo, HIGH);

  //Calculate the distance (in cm) based on the speed of sound.
  return duration[i]/58.2;
}



/* GET DATE ---------------------------------------*/
void dateTime(uint16_t* date, uint16_t* time)
{
  uint16_t year = 2014;
  uint8_t month = 4, day = 19, hour = 9, minute = 0, second = 0;

  *date = FAT_DATE(year, month, day);
  *time = FAT_TIME(hour, minute, second);
}
/* GET TIME ---------------------------------------*/
String getTime(uint32_t mls){
  String h = String(mls / 3600000);
  if(mls / 3600000 < 10) h = "0" + h;
  String m = String(mls / 60000);
  if(mls / 60000 < 10) m = "0" + m;  
  String s = String((mls / 1000) % 60);
  if((mls / 1000) % 60 < 10) s = "0" + s;
  return h + "." + m + "." + s;
}





