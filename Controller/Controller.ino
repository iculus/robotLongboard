#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>

#include <RH_RF95.h>

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

//oled
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#define LED      13

//rx
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
//915.0
#define RF95_FREQ 434.0

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
const int servoRange = SERVOMAX-SERVOMIN;
long int lookup[servoRange];
long int lookupScaled[servoRange];

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(9600);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.setBatteryVisible(true);
  Serial.println("OLED begun");
  
  delay(1000);

  // Clear the buffer.
  oled.clearDisplay();
  oled.display();

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  
  oled.setCursor(0,0);
  oled.display(); // actually display all of the above

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(12, INPUT);

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

  //build a conversion table
  for (int i = -servoRange/2; i<=servoRange/2; i++){
    int numtoinput = i*i;
    //correct fot the negative half
    if(i<0){
      numtoinput = numtoinput*-1;
    }
    lookup[i+servoRange/2] = numtoinput;
    Serial.print(i);
    Serial.print('\t');
    Serial.print (i+servoRange/2);
    Serial.print('\t');
    Serial.println(lookup[i+servoRange/2]);
    
  } 

  int maxLookup = lookup[servoRange];
  int minLookup = lookup[0];

  Serial.println();
  Serial.println(minLookup);
  Serial.print("\t");
  Serial.println(maxLookup);
  

  for (int j = 0; j<=servoRange; j++){
    lookupScaled[j] = map(lookup[j],minLookup,maxLookup,SERVOMIN,SERVOMAX);
    Serial.println(lookupScaled[j]);
  }
}



int16_t packetnum = 0;  // packet counter, we increment per xmission

int offset = -1;
int accOffset = 0;
int velocityNow = 0;
int velocityFuture = 0;
int pulseLenMid = 390 + offset;
int accStart = 5;
int accOffsetMin = -accStart+1;

void loop() {
  
  //oled battery setup
  oled.clearDisplay();
  // get the current voltage
  float battery = getBatteryVoltage();
  // update the battery icon
  oled.setBattery(battery);
  oled.renderBattery();

  //data
  int xVal = analogRead(A0);
  int yVal = analogRead(A1);
  int button = digitalRead(12);

  //map x val to servo min and max
  uint16_t pulselen = map(xVal,0,1023,SERVOMIN,SERVOMAX);
  pulselen = pulselen+offset;

  //set y val to right increase accel, left decrease
  int yInc = 5;
  int yDec = -5;
  int yValSimple = map(yVal,5,1023,yDec,yInc);
  if (yValSimple == yInc){accOffset += 1;}
  if (yValSimple == yDec && accOffset > accOffsetMin){accOffset -= 1;}
  
  //set servo limits to prevent unknown conditinos
  if (pulselen >= SERVOMAX){pulselen = SERVOMAX;}
  if (pulselen <= SERVOMIN){pulselen = SERVOMIN;}

  //make velocity % map
  int percent = map(pulselen,SERVOMIN+offset,SERVOMAX+offset,0,100);

  //add scaling
  bool scaleGo = true;
  int velThen = pulselen;
  if (scaleGo == true){
    int selection = map(pulselen-offset,SERVOMIN,SERVOMAX,0,servoRange);
    velocityNow = lookupScaled[selection]+offset;
  }
  
  //add acceleration do not use with scaling
  bool accGo = false;
  int acceleration = accStart + accOffset;
  if (accGo == true){
    velocityFuture = pulselen;
    if (velocityFuture <= velocityNow || velocityFuture < pulseLenMid){
      velocityNow = velocityFuture;
    }
    if (velocityFuture > velocityNow && velocityFuture >= pulseLenMid){
      velocityNow += acceleration;
    }
  }
  
  //display results
  oled.print("Vel  :  ");
  //oled.print('\t');
  //oled.println(percent);
  oled.print('\t');
  oled.print(velocityNow);
  oled.print('\t');
  oled.println(velThen);
  oled.print("acc  :  ");
  //oled.print('"\t");
  oled.println(acceleration);
  /*Serial.print(percent);
  Serial.print('\t');
  Serial.print(pulselen);
  Serial.print('\t');
  Serial.println(velocityNow);
  */
  //lora
  //const int radioLen = 20;
  char radiopacket[20] = "Radio-0000#        ";
  itoa(velocityNow, radiopacket+13, 10);
  radiopacket[19] = 0;

  //Serial.println(radiopacket);
  
  rf95.send((uint8_t *)radiopacket, 20);
  rf95.waitPacketSent();
   
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.waitAvailableTimeout(100))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
    {  
      //Serial.println((char*)buf);
      oled.print("Conn :  ");
      //oled.print("\t");
      //oled.print ("YES!!");
      oled.print((char*)buf);
    }
    else {Serial.println("Receive failed");}
  }
  else
  {
    oled.print("Conn :  ");
    oled.print("\t");
    oled.print ("nope");
  }

  //if (! digitalRead(BUTTON_A)) oled.print("A");
  if (! digitalRead(BUTTON_B)) offset += 2;
  if (! digitalRead(BUTTON_C)) offset -= 2;
  
  yield();
  oled.display();
}

#define VBATPIN A7

float getBatteryVoltage() {

  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;

  }
