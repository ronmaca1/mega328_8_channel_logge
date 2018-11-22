//<debuuging defines>
#define   _DEBUG_ // for serial debugging
//#undef    _DEBUG_
//</debuging defines>
#define _TIMESTAMP_PER_MINUTE_
#undef  _TIMESTAMP_PER_MINUTE_
#define _TIMESTAMP_PER_POWERUP_
//#undef _TIMESTAMP_PER_POWERUP_

//<pin defines>

#define CD4051IN        A0
#define CD4051A0        A1
#define CD4051A1        A2
#define CD4051A2        A3
#define LED_GREEN       3
#define HEAT_1          5 // digital input
#define HEAT_2          6 // digital input
#define HEAT_3          7 // digital input
#define HEAT_4          8 // digital input
#define DEBUG_HEARTBEAT 9 // tracking millis() accuracy

//</pin defines>

//<led defines>
//#define OFF             0
#define GREEN_ON    127
//</led defines>

#define B1OXYGEN        0 // 4051 mux addr 0
#define B2OXYGEN        1 // and so on
#define B3OXYGEN        2
#define B4OXYGEN        3
#define TPOS            4
#define NOTUSED         5

#include <Wire.h>
#include <stdio.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

RTC_PCF8523 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

unsigned long startmillis = 0;
unsigned long currentmillis = 0;

//  used to timestamp output to file every minute
#ifdef  _TIMESTAMP_PER_MINUTE_
unsigned char loopcount = 0;
#endif

//  cleared first trip through the loop
//  used to time stamp startup.
unsigned char powerup = 1;  

/*
long mymap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
*/
void setup() {
  // put your setup code here, to run once:
  //
  // first set adc reference to external to 
  analogReference(EXTERNAL);       // 2.5 Volt reference used
  // disable input buffers on ADC pins,
  // per datasheet page 43
  DIDR0 |= _BV(ADC0D); //adc0 is input from 4051
  // real time clock check
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    analogWrite (LED_GREEN,OFF);
    while (1);
    }
  // <set all unused pins to OUTPUT >  
  // digital pins
  int i;  
  const unsigned char unused_pins[1]{9};
  const size_t numunused = sizeof(unused_pins)/sizeof(unused_pins[0]);

    for(i=0;i<numunused;i++){
      digitalWrite(unused_pins[i],LOW);
      pinMode(unused_pins[i],OUTPUT);
      }
  // </set all unused pins to output and LOW>

  // <digital pins in use>
    
    
    pinMode (DEBUG_HEARTBEAT,OUTPUT);
    pinMode(HEAT_1,INPUT);
    pinMode(HEAT_2,INPUT);
    pinMode(HEAT_3,INPUT);
    pinMode(HEAT_4,INPUT);
  // </digital pins in use>
  
  //<pwm pins in use>
  
  analogWrite(LED_GREEN,OFF);
  //</pwm pins in use>

  //delay(20); // a short delay to let things stabilize
  #ifdef  _DEBUG_
  

  Serial.begin(57600);
  Serial.print("Initializing SD card...");
  #endif
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    #ifdef  _DEBUG_
    Serial.println("Card failed, or not present");
    // don't do anything more:
    #endif
    analogWrite(LED_GREEN,OFF); // no card inserted
   
    while (1); // hang till power down and card inserted
    
  }
  #ifdef _DEBUG_
  Serial.println("card initialized.");
  #endif
  String  StartString = "";
  StartString += "Startup";
  
  analogWrite(LED_GREEN,GREEN_ON);

  startmillis = millis();
 
}



void loop() {
  // put your main code here, to run repeatedly:

int notused, tpos, b1oxygen, b2oxygen, b3oxygen, b4oxygen, temp;



String dataString = "";
currentmillis = millis();
#ifdef  _TIMESTAMP_PER_MINUTE_

DateTime now = rtc.now();
dataString += now.unixtime();
dataString +=",";
 
#endif
#ifdef  _TIMESTAMP_PER_POWERUP_ 
powerup = 0;
DateTime now = rtc.now();
dataString += now.unixtime();
dataString +=",";

#endif
  
dataString += String(millis()-startmillis);
dataString += String(",");

 
  // <get us some heater info>
  dataString += String(digitalRead(HEAT_1));
  dataString += String(",");
  dataString += String(digitalRead(HEAT_2));
  dataString += String(",");
  dataString += String(digitalRead(HEAT_3));
  dataString += String(",");
  dataString += String(digitalRead(HEAT_4));
  dataString += String(",");
  
  // </get us some heater info>

  
  //<get us some o2 info (gain of 2(1))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp = temp >> 2;  
  b1oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b1oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(1))>
  
  //<get us some o2 info (gain of 2(2))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp = temp >> 2;
  b2oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b2oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(2))>

    //<get us some o2 info (gain of 2(3))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(B3OXYGEN);
  temp += analogRead(B3OXYGEN);
  temp += analogRead(B3OXYGEN);
  temp += analogRead(B3OXYGEN);
  temp = temp >> 2;
  b3oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b3oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(3))>

  //<get us some o2 info (gain of 2(4))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(B4OXYGEN);
  temp += analogRead(B4OXYGEN);
  temp += analogRead(B4OXYGEN);
  temp += analogRead(B4OXYGEN);
  temp = temp >> 2;
  b4oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b4oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(4))>

  
  // <get us some throttle info (gain = 1/2(1))> 
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp = temp >> 2;    
  tpos = map(temp,0,1023,0,5000);
  
  dataString += String(tpos);
  dataString += String(",");
  // </get us some throttle info>
  
  // <get us some unused channel (gain = 1/2(2))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp = temp >> 2;   
  notused = map(temp,0,1023,0,5000);
  
  dataString += String(notused); // last channel no comma appended
// </get us some unused channel (gain = 1/2(2))>

  // <SD card setup>
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
     #ifdef _DEBUG_
    // print to the serial port too:
    Serial.println(dataString);
     #endif
  }
  // if the file isn't open, pop up an error:
  else {
    #ifdef _DEBUG_
    Serial.println("error opening datalog.csv");
    #endif
    analogWrite (LED_GREEN,OFF); // off if file error
    
  }
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ enabled
  /*
  #ifdef  _DEBUG_
    delay(40);
  #endif
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ disabled 
   
  #ifndef _DEBUG_ 
    delay (100);
  #endif  
  */

while (millis()-currentmillis < 100); // do every 100 millis aka 10 sample / sec.
#ifdef _TIMESTAMP_PER_MINUTE_ 
if(loopcount <=600){
  loopcount++; 
  }else {
    loopcount = 0; //reset each minute 
  // see beginning of loop for the usage of this}
  }
#endif  
digitalWrite(DEBUG_HEARTBEAT,!digitalRead(DEBUG_HEARTBEAT));
}
