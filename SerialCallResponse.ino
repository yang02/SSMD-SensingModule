boolean serialCom = true;
const int BUFFER_LENGTH = 10;
const int SENSOR_NUM = 3;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
char APIN[SENSOR_NUM] = {
  A0, A1, A2};
  
int buffer[SENSOR_NUM][BUFFER_LENGTH];
int index = 0;

int val[SENSOR_NUM];
byte inByte = 0;  // incoming serial byte

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  if(serialCom) establishContact();  // send a byte to establish contact until receiver responds 
}

void loop()
{
  
  for(int i=0; i<SENSOR_NUM; i++){
    if(i == 0){
      int peak = getVolume(i);
      buffer[i][index] = peak;
    }
    else{
      buffer[i][index] = analogRead(APIN[i]);
    }
    val[i] = getSmoothedData(buffer[i]);
    index = (index + 1) % BUFFER_LENGTH;
  }
  
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0 && serialCom) {
    // get incoming byte:
    inByte = Serial.read();

    for(int i=0; i<SENSOR_NUM; i++){
      Serial.write(val[i] >> 8);
      Serial.write(val[i] & 255);
      delay(5);
    }
  }
  else{
    for(int i=0; i<SENSOR_NUM; i++){
      Serial.print(val[i]);
      if(i == SENSOR_NUM-1) Serial.println();
      else Serial.print(" ");
    }
  }
}

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

/* GET VOLUME */
int getVolume(int p){
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int sample;

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  while (millis() - startMillis < sampleWindow){
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

