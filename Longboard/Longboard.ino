/********************************************************************************
 * Robot Longboard by Mike Soroka                                               *
 * Updated 08/23/18                                                             *
 * This version has output for LEDs and RX/TX with LORA on M0 SAMD21G18 Feather *
 * Adding encoder feedback                                                      *
 * Adding PID with encoder feedback                                             *
 ********************************************************************************/

//Pin Defs and Consts 
#define NeoPIN A5
#define NUM_PIXELS 24
#define SERVOMIN  150 // this is the 'minimum' pulse length
#define SERVOMAX  480 // this is the 'maximum' pulse length
uint8_t servonum = 7;
#define LED 13
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 434.0 //match RX's freq!

//Notes on timers
//something something change a timer in RH_ASK.cpp
//https://forum.arduino.cc/index.php?topic=306685.0

#include <PID_v1.h>
#include <SPI.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, NeoPIN, NEO_GRB);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
RH_RF95 rf95(RFM95_CS, RFM95_INT); 


double Setpoint, Input, Output;
double Kp=2.5, Ki=0, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//for hall effect
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;
#define hallPin 12

void setup() 
{
  myPID.SetMode(AUTOMATIC);

  //hall
  attachInterrupt(digitalPinToInterrupt(hallPin), magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
   
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial);
  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  strip.begin();
  strip.setBrightness(32);
  strip.show();
}


uint16_t i;
uint32_t elapsed, t, startTime = micros();
long int updateTime = 0;
int command = 0;

void loop()
{
  if (rf95.available())
  {
    //hall
    if (millis()-updateTime > 300){
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
      //Serial.println(gotThis);
      int hashLocation = gotThis.indexOf('#');
      //Serial.println(hashLocation);
      String nums = gotThis.substring(hashLocation+1);
      //Serial.println(nums);
      int numsConvert = nums.toInt();
      //Serial.println(numsConvert);

      //uint16_t setpoint = map(numsConvert,0,1023,SERVOMIN,SERVOMAX);
      uint16_t setpoint = numsConvert;
      //Serial.println(setpoint);

      //from controller servo min = 150 max = 595 mid = 390
      //current rpm range 0::10,500

      //compare rpm to setpoint
      int rpmConvert = map(rpm,0,10500,374,599);

      //Setpoint = setpoint;
      //Input = rpmConvert;
      //myPID.Compute();

      //Output = map(Output,0,255,390,599);
      
      //command = Output;
      //Serial.println(command);
      int error = setpoint - rpmConvert;
      int maxError = 600-390; //210

      if (command < 390){command = 390;}
      if (error > 0 && setpoint > 390){command = command +5;}
      if (error <= -3) {command = setpoint;}
      if (setpoint <=390) {command = setpoint;}
      if (command >= 600) {command = 600;}

      Serial.print(command);
      Serial.print('\t');
      Serial.print(setpoint);
      Serial.print('\t');
      Serial.println(error);
                 
      pwm.setPWM(servonum, 0, command);
 
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      //delay(10);
      // Send a reply
      //delay(200); // may or may not be needed
      char data[20] = "B0#     ";
      itoa(rpmConvert,data+4,10);
      rf95.send((uint8_t *)data, 20);
      //rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
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
   //Serial.println("detect");
 }
