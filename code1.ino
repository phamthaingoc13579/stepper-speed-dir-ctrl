#include "GantryXYZ.h"

#define right_sw  13
#define left_sw   11

#define potmeterPin A0
#define encoderPinA 3
#define encoderPinB 4
#define batterySensePin A1

const float kAdcReferenceVoltage = 5.0f;
const float kAdcResolution = 1023.0f;
// Example divider: 100k (R1) to Vbat and 33k (R2) to GND -> ratio = (R1+R2)/R2.
const float kBatteryDividerRatio = (100.0f + 33.0f) / 33.0f;
const unsigned long kTelemetryIntervalMs = 500;

GantryXYZClasses gantryXYZ;

volatile boolean rightPress = false;
volatile boolean leftPress  = false;

uint8_t rightBtnState = 0;
uint8_t lastRightBtnState=0;


uint8_t leftBtnState = 0;
uint8_t lastLeftBtnState=0;

unsigned int delay_time=500;

long encoderPosition = 0;
uint8_t lastEncoderState = 0;
unsigned long lastTelemetryReport = 0;

void updateEncoder();
float readBatteryVoltage();
void reportTelemetry(float batteryVoltage);
long getEncoderPosition();

void setup() {

    pinMode(right_sw, INPUT_PULLUP);
    pinMode(left_sw, INPUT_PULLUP);
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    Serial.begin(9600);  // open coms
    gantryXYZ.setFeedrate(1000);  // set default speed

    lastEncoderState = (digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB);

}

void loop() {

    /* 
     *  Home limit switch Hall Effect sensor normal = 1
     *                                       Hit    = 0
     *                                       
     */

      updateEncoder();

      int p = analogRead(potmeterPin);
      delay_time = map(p,0,1023,1500,50) ;

      float batteryVoltage = readBatteryVoltage();

     if( !gantryXYZ.is_move){



           //   Serial.println( delay_time );
            
             checkIfRightIsPressed();
             
             checkIfLeftIsPressed();

             if( rightPress){
                 rightPress = false;
                 gantryXYZ.prepareMove(-1000,0,0);
             }
            
             else if(leftPress){
              
                  leftPress = false;
                  gantryXYZ.prepareMove( 450,0,0);
             }
     }


  // put your main code here, to run repeatedly:
    while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message

    gantryXYZ.storeBuffer( c );
  
    if(c=='\n') {
      // entire message received
      gantryXYZ.storeBuffer( c );
      Serial.print(F("\r\n"));  // echo a return character for humans
      gantryXYZ.processCommand();  // do something with the command
      
    }
  }


   gantryXYZ.move(delay_time);
   reportTelemetry(batteryVoltage);


}

void checkIfRightIsPressed()
{
  rightBtnState     = digitalRead(right_sw);
  if (rightBtnState != lastRightBtnState) 
  {
    if (rightBtnState == 0) {
         rightPress=true;
    }
    delay(50);
  }
   lastRightBtnState = rightBtnState;
}

void updateEncoder()
{
  static const int8_t encoderLookup[16] = {0, -1, 1, 0,
                                           1, 0, 0, -1,
                                           -1, 0, 0, 1,
                                           0, 1, -1, 0};

  uint8_t currentState = (digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB);
  uint8_t transition = (lastEncoderState << 2) | currentState;
  encoderPosition += encoderLookup[transition];
  lastEncoderState = currentState;
}

float readBatteryVoltage()
{
  analogRead(batterySensePin);  // Throw away the first reading after channel switch.
  int raw = analogRead(batterySensePin);
  float voltageAtPin = (raw * kAdcReferenceVoltage) / kAdcResolution;
  return voltageAtPin * kBatteryDividerRatio;
}

long getEncoderPosition()
{
  return encoderPosition;
}

void reportTelemetry(float batteryVoltage)
{
  unsigned long now = millis();
  if (now - lastTelemetryReport < kTelemetryIntervalMs) {
    return;
  }

  lastTelemetryReport = now;
  Serial.print(F("Encoder ticks: "));
  Serial.print(getEncoderPosition());
  Serial.print(F(" | Battery: "));
  Serial.print(batteryVoltage, 2);
  Serial.println(F(" V"));
}

void checkIfLeftIsPressed()
{
  leftBtnState     = digitalRead(left_sw);
  
  if (leftBtnState != lastLeftBtnState) 
  {
    if (leftBtnState == 0) {
         leftPress=true;
    }
    delay(50);
  }
   lastLeftBtnState = leftBtnState;
}
