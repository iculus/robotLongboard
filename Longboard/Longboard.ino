
#include <SPI.h>

//something something change a timer in RH_ASK.cpp
//https://forum.arduino.cc/index.php?topic=306685.0

#include <Adafruit_NeoPixel_ZeroDMA.h>

#define PIN        A5
#define NUM_PIXELS 24
Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, PIN, NEO_GRB);
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  480 // this is the 'maximum' pulse length count (out of 4096)
uint8_t servonum = 7;

//for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//for encoder
#define encoder0PinA  11
#define encoder0PinB  12
volatile long encoder0Pos=0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel;

// Change to 434.0 or 915.0 other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup() 
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial);
  Serial.begin(9600);
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

  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(11, doEncoder, RISING);  // encoDER ON PIN 2
  
  strip.begin();
  strip.setBrightness(32);
  strip.show();
  
}


uint16_t i;
// Rainbow cycle
uint32_t elapsed, t, startTime = micros();
void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      t       = micros();
      elapsed = t - startTime;
      Serial.print(elapsed);
      Serial.print('\t');
      Serial.print(t);
      Serial.print('\t');
      Serial.println(startTime);
      //if(elapsed > 5000000) break; // Run for 5 seconds
      for(i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, Wheel((uint8_t)(
          (elapsed * 256 / 1000000) + i * 256 / strip.numPixels())));
      }
      //if(elapsed >= 1000000){
      //  startTime = micros();
      //}
      strip.show();
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

      //uint16_t pulselen = map(numsConvert,0,1023,SERVOMIN,SERVOMAX);
      uint16_t pulselen = numsConvert;
      Serial.println(pulselen);

      newposition = encoder0Pos;
      newtime = millis();
      vel = (newposition-oldposition) * 1000 /(newtime-oldtime);
      Serial.print ("speed = ");
      Serial.println (vel);
      oldposition = newposition;
      oldtime = newtime;
      
      pwm.setPWM(servonum, 0, pulselen);
      

      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      //delay(10);
      // Send a reply
      //delay(200); // may or may not be needed
      uint8_t data[] = "Board-0000";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos++;
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

