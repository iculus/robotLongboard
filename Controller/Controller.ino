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
#define RF95_FREQ 915.0

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  480 // this is the 'maximum' pulse length count (out of 4096)

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(9600);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.setBatteryVisible(true);
  // init done
  Serial.println("OLED begun");
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //display.display();
  delay(1000);

  // Clear the buffer.
  oled.clearDisplay();
  oled.display();
  
  Serial.println("IO test");

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
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

int offset = -30;

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

  int cal = xVal+offset;
  uint16_t pulselen = map(cal,0,1023,SERVOMIN,SERVOMAX);
  
  oled.print(cal);
  oled.print('\t');
  oled.print(pulselen);

  
  //lora
  
  char radiopacket[20] = "HelloWorld#      ";
  itoa(cal, radiopacket+13, 10);
  radiopacket[19] = 0;
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
      oled.print ("YES!!");  
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    oled.print ("nope");
  }

  
  
  //if (! digitalRead(BUTTON_A)) oled.print("A");
  if (! digitalRead(BUTTON_B)) offset += 10;
  if (! digitalRead(BUTTON_C)) offset -= 10;
  
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
