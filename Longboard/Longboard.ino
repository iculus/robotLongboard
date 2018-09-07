/********************************************************************************
   Robot Longboard by Mike Soroka
   Updated 09/06/18
   This version has output for LEDs and RX/TX with LORA on M0 SAMD21G18 Feather
   With encoder feedback
   With coast mode
   With logging serial
 ********************************************************************************/

//Notes on timers
//something something change a timer in RH_ASK.cpp
//https://forum.arduino.cc/index.php?topic=306685.0

#include <SPI.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "names.h"

Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, NeoPIN, NEO_GRB);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup()
{
  //serial coms to logger
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(LED, OUTPUT);
  delay(100);

  //hall setup area
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), magnet_detect, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;

  //for servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //for radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW); delay(10);   // manual reset
  digitalWrite(RFM95_RST, HIGH); delay(10);  // manual rese
  while (!rf95.init()) {Serial.println("LoRa radio init failed");while (1);}
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {Serial.println("setFrequency failed");while (1);}
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:

  strip.begin();
  strip.setBrightness(32);
  strip.show();
}


uint16_t i;
uint32_t elapsed, t, startTime = micros();
long int updateTime = 0;
int command = 0;
String holder = "";
void loop()
{
  //from slave logger
  if (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    if (inChar == '\n') {stringComplete = true;}
  }
  if (stringComplete) {
    if (inputString.substring(0, 5) == "$MSRS") {
      int pipeLoc = (inputString.indexOf('|'));
      int check = inputString.substring(6, pipeLoc).toInt();
      int measured = (inputString.length() - pipeLoc - 1);
      if (measured == check) {holder = inputString;}
    }
    // clear the string:
    inputString = "";stringComplete = false;
  }

  //from radio
  digitalWrite(LED, LOW);
  if (rf95.available())
  {
    //hall 0Vel reset
    if (millis() - updateTime > 200) {rpm = 0;}
    if (half_revolutions >= 5) {
      rpm = 30 * 1000 / (millis() - timeold) * half_revolutions;
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
      String nums = gotThis.substring(hashLocation + 1);
      int numsConvert = nums.toInt();
      setpoint = numsConvert;

      //from controller servo min = 150 max = 599 mid = 390 current rpm range 0::10,500
      rpmConvert = map(rpm, 0, 10500, neutral, setpointMax);
      if (setpoint >= deadbandMin && setpoint <= deadbandMax && setpoint != 0) {coast = true;control = false;}
      if (setpoint < deadbandMin || setpoint > deadbandMax && setpoint != 0) {coast = false;control = true; }
      if (control && not coast) {
        if (setpoint > deadbandMax) {go = true;brake = false;}
        if (setpoint < deadbandMin) {go = false;brake = true;}}

      /*now we know coast, control, go, brake*/

      if (brake && setpoint < brakeSetPt) {softBrake = false;hardBrake = true;}
      if (brake && setpoint >= brakeSetPt) {softBrake = true;hardBrake = false;}
      if (go) {softBrake = false;hardBrake = false;}

      /*now we know hard and soft brake*/

      int range = 2; //to set a soft coast deadband control range
      if (not control && coast) {
        if (rpmConvert > neutral + range) {movement = true;stationary = false;}
        if (rpmConvert <= neutral + range) {movement = false;stationary = true;}}

      /*now we know board status (movement and stationary)*/

      error = setpoint - rpmConvert;

      if (coast) {
        command = rpmConvert;hasBeenSoft = false;softBrake = false;hardBrake = false;brake = false;go = false;}
      if (control && go) {
        hasBeenSoft = false; //resets flag for soft braking
        if (command < setpoint && command >= rpmConvert)  {command = command + posAccel;} //if the setpoint is above the vehicle command rpm
        //below has been added to set coast rpm when command is active
        if (command < setpoint && command < rpmConvert)  {command = rpmConvert;} //if the setpoint is above the vehicle command rpm
        if (command >= setpoint) {command = setpoint;} //if setpoint falls below the vehicle command rpm
        if (command >= setpointMax) {command = setpointMax;}} //clamp the value below the maximum
        
      /*now we have a value for control in go mode and coast mode*/

      if (control && hardBrake) {
        if (not hasBeenSoft) {command = neutral;hasBeenSoft = true;}
        if (command <= setpoint) {command = setpoint;} //setpoint value clamping
        if (command > setpoint) {command = command - negAccelHard;}}
      if (control && softBrake) {
        if (not hasBeenSoft) {command = neutral;hasBeenSoft = true;}
        if (command <= setpoint) {command = setpoint;} //setpoint value clamping
        if (command > setpoint) {command = command - negAccel;}}

      /*now we have a value for control in hard and soft barking mode*/

      //unused for the e-brake
      if (setpoint == 0) {control = true;command = 152;}

      /*now we have a value for the e-brake*/

      String TXpre =
        hControl + control + ',' +
        hCoast + coast + ',' +
        hBrake + brake + ',' +
        hSBrake + softBrake + ',' +
        hHBrake + hardBrake + ',' +
        hGo + go + ',' +
        hMove + movement + ',' +
        hSation + stationary + ',' +
        hCommand + command + ',' +
        hSet + setpoint + ',' +
        hErr + error + ',' +
        hRpm + rpmConvert;

      int sLen = TXpre.length();
      String TX = header + ',' + sLen + '|' + TXpre;
      Serial1.println(TX);

      printToScreen();

      if (control) {pwm.setPWM(servonum, 0, command);}
      if (coast) {pwm.setPWM(servonum, 0, neutral);}

      //Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC); delay(10);

      // Send a reply
      char data[20] = "B0#     ";
      itoa(rpmConvert, data + 4, 10);
      rf95.send((uint8_t *)data, 20);
      //rf95.waitPacketSent(); Serial.println("Sent a reply");
      //digitalWrite(LED, LOW);

      /*
        // Send a reply
        //char data[300] = "B0#     ";
        //itoa(rpmConvert,data+4,10);
        const char* data = holder.c_str();
        rf95.send((uint8_t *)data, 220);
        //rf95.waitPacketSent(); Serial.println("Sent a reply");
        //digitalWrite(LED, LOW);
      */
    }
    else
    {Serial.println("Receive failed");}
  }
  //out of radio
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);}
  if (WheelPos < 170) {WheelPos -= 85;return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);}
  WheelPos -= 170;return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void printToScreen(){
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
}

void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
{half_revolutions++;updateTime = millis();}
