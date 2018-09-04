/********************************************************************************
 * Robot Longboard by Mike Soroka                                               *
 * Updated 08/28/18                                                             *
 * This version has output for LEDs and RX/TX with LORA on M0 SAMD21G18 Feather *
 * With encoder feedback                                                        *
 * With coast mode                                                              *
 ********************************************************************************/

//Pin Defs and Consts 
#define NeoPIN A5
#define NUM_PIXELS 24
#define SERVOMIN  150 // this is the 'minimum' pulse length
#define SERVOMAX  480 // this is the 'maximum' pulse length
int servonum = 7;
#define LED 13
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
//#define RF95_FREQ 434.0 //match RX's freq!
#define RF95_FREQ 915.0 //match RX's freq!
#define hallPin 12



//Notes on timers
//something something change a timer in RH_ASK.cpp
//https://forum.arduino.cc/index.php?topic=306685.0

#include <SPI.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, NeoPIN, NEO_GRB);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
RH_RF95 rf95(RFM95_CS, RFM95_INT); 

//for hall effect
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

//set up for calcs
int setpointMax = 599;
int setpointMin = 152;
int neutral = 374;
int posAccel = 5;
int negAccel = 15;
int negAccelHard = 25;
int deadbandMin = 360;
int deadbandMax = 380;
int brakeSetPt = 175;
bool coast = false;
bool control = false;
bool brake = false;
bool go = false;
bool movement = false;
bool stationary = false;
bool softBrake = false;
bool hardBrake = false;
bool hasBeenSoft = false;
//int 12bit = 4096;
//int 10bit = 1024;

void setup() 
{  
  //serial coms to logger
  Serial1.begin(115200);
  Serial.begin(115200); delay(100);
  pinMode(LED, OUTPUT);
  
  //hall setup area 
  pinMode(hallPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), magnet_detect, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
   
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  while (!rf95.init()) {Serial.println("LoRa radio init failed"); while (1);}
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {Serial.println("setFrequency failed"); while (1);}
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  strip.begin();
  strip.setBrightness(32);
  strip.show();

  analogReadResolution(12);
}


uint16_t i;
uint32_t elapsed, t, startTime = micros();
long int updateTime = 0;
int command = 0;

void loop()
{
  digitalWrite(LED, LOW);
  if (rf95.available())
  {
    //hall 0Vel reset
    if (millis()-updateTime > 200){
      rpm = 0;
    }
    
    if (half_revolutions >= 5) { 
      rpm = 30*1000/(millis() - timeold)*half_revolutions;
      timeold = millis();
      half_revolutions = 0;
    }
   
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      /*
      t = micros();
      elapsed = t - startTime;
      //Serial.print(elapsed);
      //Serial.print('\t');
      //Serial.print(t);
      //Serial.print('\t');
      //Serial.println(startTime);
      //if(elapsed > 5000000) break; // Run for 5 seconds
      for(i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, Wheel((uint8_t)(
          (elapsed * 256 / 1000000) + i * 256 / strip.numPixels())));
      }
      strip.show();
      */
      
      digitalWrite(LED, HIGH);
      //RH_RF95::printBuffer("Received: ", buf, len);
      //Serial.print("Got: ");
      //Serial.println((char*)buf);
      
      String gotThis = (char*)buf;
      int hashLocation = gotThis.indexOf('#');
      String nums = gotThis.substring(hashLocation+1);
      int numsConvert = nums.toInt();
      uint16_t setpoint = numsConvert;

      //from controller servo min = 150 max = 599 mid = 390 current rpm range 0::10,500
      int rpmConvert = map(rpm,0,10500,neutral,setpointMax);
      if (setpoint >= deadbandMin && setpoint <= deadbandMax && setpoint != 0) {coast = true; control = false;}
      if (setpoint < deadbandMin || setpoint > deadbandMax && setpoint != 0) {coast = false; control = true;}
      if (control && not coast) {
        if (setpoint > deadbandMax) {go = true; brake = false;}
        if (setpoint < deadbandMin) {go = false; brake = true;}
      }

      /*now we know coast, control, go, brake*/

      if (brake && setpoint < brakeSetPt){softBrake = false; hardBrake = true;}
      if (brake && setpoint >= brakeSetPt){softBrake = true; hardBrake = false;}
      if (go) {softBrake = false; hardBrake = false;}

      /*now we know hard and soft brake*/
     
      int range = 2; //to set a soft coast deadband control range 
      if (not control && coast){
        if (rpmConvert > neutral+range) {movement = true; stationary = false;}
        if (rpmConvert <= neutral+range) {movement = false; stationary = true;}
      }

      /*now we know board status (movement and stationary)*/
       
      int error = setpoint - rpmConvert;

      if (coast){command = rpmConvert; hasBeenSoft = false; softBrake = false; hardBrake = false; brake = false; go = false;}
      if (control && go){
        hasBeenSoft = false; //resets the desired flag for soft braking
        if (command < setpoint){command = command + posAccel;} //if the setpoint is above the vehicle command rpm
        if (command >= setpoint) {command = setpoint;} //if setpoint falls below the vehicle command rpm
        if (command >= setpointMax) {command = setpointMax;} //clamp the value below the maximum
      }

      /*now we have a value for control in go mode and coast mode*/ 
      
      if (control && hardBrake) {
        if (not hasBeenSoft) {command = neutral; hasBeenSoft = true;}
        if (command <= setpoint) {command = setpoint;} //for overflow command calcs and setpoint value clamping
        if (command > setpoint) {command = command - negAccelHard;}
      }
      if (control && softBrake) {
        if (not hasBeenSoft) {command = neutral; hasBeenSoft = true;}
        if (command <= setpoint) {command = setpoint;} //for overflow command calcs and setpoint value clamping
        if (command > setpoint) {command = command - negAccel;}
      }

      /*now we have a value for control in hard and soft barking mode*/
      
      if (setpoint == 0){control = true; command = 152;} //for the e-brake

      /*now we have a value for the e-brake*/

      String header = "$MSR";
      String hControl = "%C";
      String hCoast = "%O";
      String hBrake = "%B";
      String hSBrake = "%S";
      String hHBrake = "%H";
      String hGo = "%G";
      String hMove = "%M";
      String hSation = "%T";
      String hCommand = "$D";
      String hSet = "%E";
      String hErr = "%R";
      String hRpm = "%P";

      String TXpre=
             hControl+control+','+
             hCoast+coast+','+
             hBrake+brake+','+
             hSBrake+softBrake+','+
             hHBrake+hardBrake+','+
             hGo+go+','+
             hMove+movement+','+
             hSation+stationary+','+
             hCommand+command+','+
             hSet+setpoint+','+
             hErr+error+','+
             hRpm+rpmConvert;

      int sLen = TXpre.length();
      String TX = header+','+sLen+'|'+TXpre;
      Serial1.println(TX);

      /*
      //Serial print the results of all vehicle states and measured, calculated, and commanded values
      Serial.print("control: ")    ;Serial.print(control);
      Serial.print("\t coast: ")   ;Serial.print(coast);
      Serial.print("\t brake: ")   ;Serial.print(brake);
      Serial.print("\t Sbrake: ")  ;Serial.print(softBrake);
      Serial.print("\t Hbrake: ")  ;Serial.print(hardBrake);
      Serial.print("\t hbs: ")     ;Serial.print(hasBeenSoft);
      Serial.print("\t go: ")      ;Serial.print(go);
      Serial.print("\t Move: ")    ;Serial.print(movement);
      Serial.print("\t Station: ") ;Serial.print(stationary);
      Serial.print("\t cmd: ")     ;Serial.print(command);
      Serial.print("\t set: ")     ;Serial.print(setpoint);
      Serial.print("\t Err: ")     ;Serial.print(error);
      Serial.print("\t rpm: ")     ;Serial.println(rpmConvert);
      */
      
      if (control){pwm.setPWM(servonum, 0, command);}
      if (coast) {pwm.setPWM(servonum, 0, neutral);}
 
      //Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC); delay(10);
      
      // Send a reply
      char data[20] = "B0#     ";
      itoa(rpmConvert,data+4,10);
      rf95.send((uint8_t *)data, 20);
      //rf95.waitPacketSent(); Serial.println("Sent a reply");
      //digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

 void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   half_revolutions++;
   updateTime = millis();
 }
