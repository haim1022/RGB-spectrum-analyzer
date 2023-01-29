#include <IRremote.h>

#define micPin A0
#define buttonPin 12
#define dataPin 11
#define latchPin 10
#define clockPin 9
#define IRPin 8

#define resetCalibrationTime 200 //Reset microphone calibration every (ms)
#define mapRange 20
#define micSampleInterval 30 //(ms)

#define IR_ON_VAL 0xBF40FF00
#define IR_OFF_VAL 0xBE41FF00
#define IR_COLOR_VAL 0xA25DFF00

unsigned long lastCalibrationReset = 0, rawIRData, lastMicSampleTime = 0, lastAnimationChange = 0;
//                          Blue,       Green,      Turquoise,  Red,        Purple,     Yellow,     White,
uint32_t IRColorCodes[7] = {0xC01DFF00, 0xC02DFF00, 0xC03DFF00, 0xC04DFF00, 0xC05DFF00, 0xC06DFF00, 0xC07DFF00};
unsigned int minRange = 4095, maxRange = 0, animationInterval = 0;
byte LEDColor = 1, rawMicVal, lastMicVal = 1, micVal = 1, LEDVal = 1;
bool lastButtonState = LOW, buttonState, systemState = HIGH;

void setup()
{
  Serial.begin(2000000);
  pinMode(micPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(IRPin, INPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  IrReceiver.begin(IRPin, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
}

void loop()
{  
  //Get microphone value
  if((millis() - lastMicSampleTime) > micSampleInterval)
  {
    rawMicVal = analogRead(micPin);
    lastMicSampleTime = millis();

    //Handle microphone calibration - Any map limit change affects both min and max to preserve the range
    if((millis() - lastCalibrationReset) > resetCalibrationTime)
    {
      minRange = rawMicVal;
      maxRange = rawMicVal+mapRange/2;
      lastCalibrationReset = millis();
    }
    if(rawMicVal < minRange)
    {
      minRange = rawMicVal-1;
      maxRange = minRange+mapRange;
    }
    if(rawMicVal > maxRange)
    {
      maxRange = rawMicVal+1;
      minRange = maxRange-mapRange;
    }
  
    //Map microphone value
    animationInterval = micSampleInterval/abs(lastMicVal-micVal);
    micVal = map(rawMicVal, minRange, maxRange, 1, 6);
  }
  else //Handle smooth animation
  {
    if((millis() - lastAnimationChange) > animationInterval)
    {
      if(lastMicVal < micVal)
        lastMicVal++;
      else if(lastMicVal > micVal)
        lastMicVal--;
      lastAnimationChange = millis();
    }
  }

  //Convert binary to spectrum analyzer steps
  LEDVal = 0x01;
  for(byte loopCounter = 1; loopCounter < lastMicVal; loopCounter++)
  {
    LEDVal = LEDVal << 1;
    LEDVal = LEDVal | 0x01;
  }

  //Transfer data to shift registers
  digitalWrite(latchPin, LOW);
  if((LEDColor & B00000001) && systemState) //LEDColor LSB is blue
    shiftOut(dataPin, clockPin, MSBFIRST, (~LEDVal)&B111111);
  else
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
  if((LEDColor & B00000010) && systemState) //Green
    shiftOut(dataPin, clockPin, MSBFIRST, (~LEDVal)&B111111);
  else
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
  if((LEDColor & B00000100) && systemState) //LEDColor MSB is red
    shiftOut(dataPin, clockPin, MSBFIRST, (~LEDVal)&B111111);
  else
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
  digitalWrite(latchPin, HIGH);

  //Decode IR for remote control
  if(IrReceiver.decode())
  {
    IrReceiver.resume(); // Enable receiving of the next value
    rawIRData = IrReceiver.decodedIRData.decodedRawData;
    Serial.print("IR: "); Serial.println(rawIRData);
    if(rawIRData == IR_OFF_VAL)
    {
      systemState = LOW;
      rawIRData = 0x00; // Reset data to prevent unwanted change
    }
    else if(rawIRData == IR_ON_VAL)
    {
      systemState = HIGH;
      rawIRData = 0x00; // Reset data to prevent unwanted change
    }
    for(byte colorCounter = 0; colorCounter < 7; colorCounter++)
    {
      if(rawIRData == IRColorCodes[colorCounter])
        LEDColor = colorCounter+1;
    }
  }

  //Handle button/IR input
  lastButtonState = buttonState;
  buttonState = digitalRead(buttonPin);
  if(((buttonState != lastButtonState && buttonState == HIGH) || (rawIRData == IR_COLOR_VAL)) && systemState)
  {
    rawIRData = 0x00; // Reset data to prevent unwanted change
    LEDColor++;
    if(LEDColor > 7)
      LEDColor = 1;
  }
}
