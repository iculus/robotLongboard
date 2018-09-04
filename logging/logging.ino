//for serial 2
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler() {Serial2.IrqHandler();}
char d = 'null';

//for gps
#include <Adafruit_GPS.h>
#define GPSSerial Serial2 // what's the name of the hardware serial port?
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port
#define GPSECHO false
uint32_t timer = millis();

//for bno055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
//for stringBuilding
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);

  //gps
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);  
  GPSSerial.println(PMTK_Q_RELEASE);
  
  //bno055
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  //string building
  inputString.reserve(200);
}
 
void loop() {
  //gps
  char c = GPS.read();
  //if (GPSECHO)
  //  if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  //bno055
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  //from master
  if (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    if (inChar == '\n') {stringComplete = true;}
  }
  if (stringComplete) {
    if (inputString.substring(0,4) == "$MSR"){
      int pipeLoc = (inputString.indexOf('|'));
      int check = inputString.substring(5,pipeLoc).toInt();
      int measured = (inputString.length()-pipeLoc-3);

      if (measured == check){
        Serial.println(inputString);
      }
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  String header = "$MSR";
  String hAx = "%AX"; float AX = (event.orientation.x); //accel
  String hAy = "%AY"; float AY = (event.orientation.y); //accel
  String hAz = "%AZ"; float AZ = (event.orientation.z); //accel 
  String hGh = "%GH"; int GH = GPS.hour;         //hour
  String hGm = "%GM"; int GM = GPS.minute;       //min
  String hGs = "%GS"; int GS = GPS.seconds;      //sec
  String hGi = "%GI"; int GI = GPS.milliseconds; //milli
  String hGd = "%GD"; int GD = GPS.day;          //day
  String hGo = "%GO"; int GO = GPS.month;        //month
  String hGy = "%GY"; int GY = GPS.year;         //year
  String hGf = "%GF"; int GF = GPS.fix;          //fix
  String hGq = "%GQ"; int GQ = GPS.fixquality;//quality
  String hGl = "%GL"; double GL = GPS.latitude;//lat
  String hGg = "%GG"; double GG = GPS.longitude;//long
  String hGp = "%GP"; float GP = GPS.speed;//speed
  String hGt = "%GT"; float GT = GPS.altitude;//alt
  String hGn = "%GN"; float GN = GPS.angle;//ang
  String hGa = "%GA"; int GA = GPS.satellites;//sats
  
  String TXpre=
     hAx+dtostrf(AX,+','+
     hAy+AY+','+
     hAz+AZ+','+
     hGh+GH+','+
     hGm+GM+','+
     hGs+GS+','+
     hGi+GI+','+
     hGd+GD+','+
     hGo+GO+','+
     hGy+GY+','+
     hGf+GF+','+
     hGq+GQ+','+
     hGl+GL+GPS.lat+','+
     hGg+GG+GPS.lon+','+
     hGp+GP+','+
     hGt+GT+','+
     hGn+GN+','+
     hGa+GA;
     
  int sLen = TXpre.length();
  String TX = header+','+sLen+'|'+TXpre;
  Serial.println(TX);
  /*
  String TXpre=
       hAx+event.orientation.x+','+
       hAy+event.orientation.y+','+
       hAz+event.orientation.z+','+
       hGh+GPS.hour+','+
       hGm+GPS.minute+','+
       hGs+GPS.seconds+','+
       hGi+GPS.milliseconds+','+
       hGd+GPS.day+','+
       hGo+GPS.month+','+
       hGy+GPS.year+','+
       hGf+GPS.fix+','+
       hGq+GPS.fixquality+','+
       hGl+GPS.latitude+GPS.lat+','+
       hGg+GPS.longitude+GPS.lon+','+
       hGp+GPS.speed+','+
       hGt+GPS.altitude+','+
       hGn+GPS.angle+','+
       hGa+GPS.satellites;

  int sLen = TXpre.length();
  String TX = header+','+sLen+'|'+TXpre;
  Serial.println(TX);
  */
  if (millis() - timer > 200) {
    timer = millis(); // reset the timer
    Serial.println("");
    Serial.print("\tX: ") ;Serial.print(AX,4);
    Serial.print("\tY: ") ;Serial.print(AY,4);
    Serial.print("\tZ: ") ;Serial.print(AZ,4);
    Serial.println("");
    Serial.print("\nTime: ");
    Serial.print(GH); Serial.print(':');
    Serial.print(GM); Serial.print(':');
    Serial.print(GS); Serial.print('.');
    Serial.println(GI);
    Serial.print("Date: ");
    Serial.print(GD); Serial.print('/');
    Serial.print(GO); Serial.print("/20");
    Serial.println(GY);
    Serial.print("Fix: "); Serial.print(GF);
    Serial.print(" quality: "); Serial.println(GQ);
    if (GF) {
      Serial.print("Location: ");
      Serial.print(GL,6); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GG,6); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GP);
      Serial.print("Angle: "); Serial.println(GN);
      Serial.print("Altitude: "); Serial.println(GT);
      Serial.print("Satellites: "); Serial.println(GA);
    }
  }
}
