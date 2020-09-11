/*
  Interface to Sharp GP2Y1010AU0F Particle Sensor
  Modified from Program by Christopher Nafis
  Written April 2012

  Changes (Sept 2016):
  ISR code for ADC

  Changes 20170701:
  Added XDATA stuff - Dan Bowen
  Added xon/xoff stuff
*/
#define codeVersion 21.3
#include <stdlib.h>
#define xon 0x11
#define xoff 0x13
const byte dustPin = 5;
// Variables that can be changed by an ISR must be declared as volatile
volatile int adcReading;
volatile boolean adcDone;
boolean adcStarted;


int ledPower = 12;
int delayTime = 280;
int delayTime2 = 40;
float offTime = 9580;
unsigned long interval = 1000; // the time (milliseconds) we need to wait between serial XDatas
unsigned long XonXoffTimeout = 5000; //time (milliseconds) to wait on xon after xoff
unsigned long previousMillis = 0; // millis() returns an unsigned long.
unsigned long currentMillis = 0;
unsigned long StoppedTxMillis = 0;

int dustVal = 0;
int i = 0;
int c = 0;
int adcMin = 1023;
int adcMax = 0;
int integerVoltage = 0;
//float ppm = 0.0;
char s[32];
float voltage = 0.0;
float vMin = 0.0;
float vMax = 0.0;
float vAvg = 0.0;
float ppmpercf = 0.0;
float dustDensity = 0.0;
char buffer[26];

//// https://www.arduino.cc/en/Tutorial/Smoothing
// Define the number of samples to keep track of.  The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.  Using a constant rather than a normal variable lets
// use this value to determine the size of the readings array.
const int numReadings = 100;  // Typically one per 10mS

unsigned int readings[numReadings];      // the readings from the analog input
unsigned int readIndex = 0;              // the index of the current reading
unsigned long total = 0;                  // the running total
unsigned int average = 0;                // the average

void setup() {
  pinMode(ledPower, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(38400);
  Serial2.begin(9600);
  Serial.print("Starting");
  delay(1000);
  i = 0;
  c = 0;
 // ppm = 0.0;
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits). this also sets ADLAR (left-adjust result)
  // to 0 (the default).
  ADMUX = bit (REFS0) | (dustPin & 0x07);

  //Clear the averaging array
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0; ////
  }
}

// ADC complete Interupt Service Routine
ISR (ADC_vect)
{
  byte low, high;
  // read the result registers and store value in adcReading
  // Must read ADCL first
  low = ADCL;
  high = ADCH;
  adcReading = (high << 8) | low;
  adcDone = true;
}
// end of ADC_vect


void loop() {
  do {
    i = i + 1;
    digitalWrite(ledPower, LOW); // power on the LED
    delayMicroseconds(delayTime); // wait 280uS
    // Check the conversion hasn't been started already
    if (!adcStarted)
    {
      adcStarted = true;
      // start the conversion
      ADCSRA |= bit (ADSC) | bit (ADIE);
    }
    delayMicroseconds(delayTime2); // wait another 40uS or so to give 320uS pulse width
    digitalWrite(ledPower, HIGH); // turn the LED off
    // give the ADC time to complete then process the reading
    delayMicroseconds(100);

    if (adcDone)
      // adcDone is set to True by the ISR, called when the conversion is complete.
    {
      adcStarted = false;
      // subtract the last reading:
      total = total - readings[readIndex];  ////
      // read from the sensor:
      readings[readIndex] = adcReading;
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;
      // if we're at the end of the array...
      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }
      // calculate the average:
      average = total / numReadings;
      if (adcMin > adcReading) adcMin = adcReading; // x second Minimum
      if (adcMax < adcReading) adcMax = adcReading; // x second Maximum
      c = c + 1; // count of readings
      adcDone = false;
    }

    delayMicroseconds(offTime); // wait for the remainder of the 10mS cycle time

// check if "xdata send interval" time has passed ( milliseconds)
 currentMillis = millis(); // grab current time
  } while ((unsigned long)(currentMillis - previousMillis) <= interval); 
  

 

 

    // Convert ADC max/min readings in last X seconds (0-1023) to voltages
  vMin = adcMin * 5.0 / 1024.0;
  vMax = adcMax * 5.0 / 1024.0;
  vAvg = average * 5.0 / 1024.0;
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
     dustDensity = (0.17 * vAvg - 0.1) * 1000;
  if (dustDensity > 0.5)
    dustDensity = 0.5;
  if (dustDensity < 0 )
    dustDensity = 0;
    integerVoltage = (int)(vAvg * 1000);  // Goes in cell current ozonesonde var
    //integerDensity = (int)(dustDensity * 100) ;  //goes in cell temperature ozonesonde var

    int integerCodeVersion = (int)(codeVersion*100);
    sprintf(buffer, "xdata=0100%04x%04x0000\r\n", integerVoltage, integerCodeVersion);
    SerialWithFlowControl();
    // save the "current" time
    previousMillis = millis();
    /* String dataString = "vMin:";
      dataString += dtostrf(vMin, 9, 4, s);
      Serial.println(dataString);
      dataString = "vMax:";
      dataString += dtostrf(vMax, 9, 4, s);
      Serial.println(dataString);
      dataString = "average:";
      dataString += dtostrf(average, 9, 4, s);
      Serial.print(dataString);
    */
    String dataString = "vAvg [v]:";
    dataString += dtostrf(vAvg, 9, 4, s);
    dataString += " DustDensity [ug/m^3]:";
    dataString += dtostrf(dustDensity, 5, 4, s);
    Serial.println(dataString);
  

  i = 0;
  c = 0;
  adcMin = 1023;
  adcMax = 0;

}


void SerialWithFlowControl() {
  int index = 0;
  bool OKtoTransmit = true;  //Start with it being OK to transmit serial.
  char inputByte = 0;
  char lengthBuffer = 0  ;
  lengthBuffer = strlen(buffer);



  while (index < lengthBuffer) {

    ////Check to see if there's any serial characters waiting to be read in from iMet
    Serial2.flush();  // wait for transmitting chars to leave
    if (Serial2.available()) {
      inputByte = Serial2.read();
      //Serial.println("InputByte:");
      //Serial.println(inputByte, DEC);
      switch (inputByte) {
        case xon:   // Received a request to start transmitting again
          {
            OKtoTransmit = true;
            Serial.println("Ok to transmit");
          }
          break;
        case xoff:   //Received a request to stop transmitting
          {
            OKtoTransmit = false;
            Serial.println("NOT Ok to transmit");
            StoppedTxMillis = millis(); // grab current time
          }
          break;
        default:
          // must be some other char, and we don't care in that case.

        break;
      }
    }
    if (OKtoTransmit) {
      digitalWrite(13, !digitalRead(13));  //Just toggle the onboard LED to see that something is happening
      Serial2.print(buffer[index]);
      index++;
    } 
     else {
        currentMillis = millis(); // grab current time
        // check if "XonXoffTimeout" time has passed since being told to stop transmitting, 
        // and keep going anyway if so.
        if ((unsigned long)(currentMillis - StoppedTxMillis) >= XonXoffTimeout) {
            Serial.println("Gave up on waiting for Xon");
            // Just exit the data sending routine.
            return;
          }
        }
     }
  Serial.println(buffer);  // Print to computer so the programmer can see.
}



