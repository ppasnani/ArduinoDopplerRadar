//generalized wave freq detection with 38.5kHz sampling rate and interrupts
//by Amanda Ghassaei
//http://www.instructables.com/id/Arduino-Frequency-Detection/
//Sept 2012

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
*/


//clipping indicator variables
boolean clipping = 0;

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;//keeps time and sends vales to store in timer[] occasionally

unsigned int period;//storage for period of wave
float frequency;//storage for frequency calculations
float velocity = 0;

//74HC595 Shift Register vars
//Pin connected to latch pin (ST_CP) of 74HC595
const int latchPin = 8;
//Pin connected to clock pin (SH_CP) of 74HC595
const int clockPin = 12;
////Pin connected to Data in (DS) of 74HC595
const int dataPin = 11;
//Pins for the 7(actually 6) segment
const int bPin = 2;
const int cPin = 4;
const int dPin = 3;
const int ePin = 5;
const int fPin = 6;
const int gPin = 10;

const int resetSwitchPin = 9;
const int unitSwitchPin = 7;

int maxVelocity = 0;

//Lopping integer to set pins high one by one
byte dataArray[10];

void setup(){
  
  Serial.begin(9600);
  
   //set pins to output because they are addressed in the main loop
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  
  pinMode(bPin, OUTPUT);
  pinMode(cPin, OUTPUT);
  pinMode(dPin, OUTPUT);
  pinMode(ePin, OUTPUT);
  pinMode(fPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  
  pinMode(resetSwitchPin, INPUT);
  pinMode(unitSwitchPin, INPUT);
  
  dataArray[0] = 0x04; //00000100
  dataArray[1] = 0xDC; //11011100
  dataArray[2] = 0x82; //10000010
  dataArray[3] = 0x88; //10001000
  dataArray[4] = 0x58; //01011000
  dataArray[5] = 0x28; //00101000
  dataArray[6] = 0x20; //00100000
  dataArray[7] = 0x9C; //10011100
  dataArray[8] = 0x00; //00000000
  dataArray[9] = 0x18; //00011000
  
  cli();//diable interrupts
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  sei();//enable interrupts
}

ISR(ADC_vect) {//when new ADC value ready
  
  PORTB &= B11101111;//set pin 12 low
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  if (prevData < 60 && newData >=60){//if increasing and crossing midpoint
    period = time;
    time = 0;    
  }
  
  time++;
}

void loop(){
  
  //if (ne>ampThreshold){
    int resetSwitchVal = digitalRead(resetSwitchPin);
    int unitSwitchVal = digitalRead(unitSwitchPin);
    
    frequency = 38462/float(period);//calculate frequency timer rate/period
  
    //print results
    //Serial.print(frequency);
    //Serial.print(" hz ");
    velocity = (frequency / 19.49);
    //Serial.print(velocity);
    //Serial.println(" km/hr");
    
    int roundVelocity = velocity;
    
    if (roundVelocity > maxVelocity){
      maxVelocity = roundVelocity;
    }
    
    int cmsVelocity = (maxVelocity * 27.8); //COnvert km/h to cm/s 
    
    if (unitSwitchVal == 1){
      displayNum(cmsVelocity);
    }else{
      displayNum(maxVelocity);
    }
    
    if (resetSwitchVal == 1){
      maxVelocity = 0; 
    }
  
  delay(100);//delete this if you want
  
  //do other stuff here
}


// This method sends bits to the shift register:

void registerWrite(byte bitsToSend) {
  //int whichPin, int whichState
// the bits you want to send
  //byte bitsToSend = 255;

  // turn off the output so the pins don't light up
  // while you're shifting bits:
  digitalWrite(latchPin, LOW);

  // turn on the next highest bit in bitsToSend:
  //bitWrite(bitsToSend, whichPin, whichState);

  // shift the bits out:
  shiftOut(dataPin, clockPin, MSBFIRST, bitsToSend);

    // turn on the output so the LEDs can light up:
  digitalWrite(latchPin, HIGH);

}

void displayNum(int numDisplay){
  //Got the numbers from just a diagram
  int lowerNum = numDisplay % 10;
  int higherNum = 0;
  if (numDisplay >= 10){
    higherNum = ((numDisplay - lowerNum) / 10);
  }
  int highASegment = (dataArray[higherNum] >> 1) & 1;
  
  registerWrite(dataArray[lowerNum] + highASegment);
  /*digitalWrite(dPin, LOW);*/
  digitalWrite(bPin, (dataArray[higherNum] >> 2) & 1);
  digitalWrite(cPin, (dataArray[higherNum] >> 3) & 1);
  digitalWrite(dPin, (dataArray[higherNum] >> 4) & 1);
  digitalWrite(ePin, (dataArray[higherNum] >> 5) & 1);
  digitalWrite(fPin, (dataArray[higherNum] >> 6) & 1);
  digitalWrite(gPin, (dataArray[higherNum] >> 7) & 1);

}
