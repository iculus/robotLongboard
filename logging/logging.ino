//for sd
#include <SPI.h>
#include <SD.h>
const int chipSelect = 4;

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

  //sd card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //while (1);
  }
  if (SD.begin(chipSelect)) {
    Serial.println("card initialized.");
  }

  //gps
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
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
  //inputString.reserve(200);

}

String holder = "";

void loop() {
  //bno055
  /* Get a new sensor event */ 

  /*
  Serial.print("EULER: "); 
  Serial.print("X: ");Serial.print(EX);
  Serial.print(" Y: ");Serial.print(EY); 
  Serial.print(" Z: ");Serial.print(EZ);Serial.println("");
  Serial.print("MAGS: ");
  Serial.print("X: ");Serial.print(MX);
  Serial.print(" Y: ");Serial.print(MY);
  Serial.print(" Z: ");Serial.print(MZ);Serial.println("");
  Serial.print("GYRO: ");
  Serial.print("X: ");Serial.print(YX);
  Serial.print(" Y: ");Serial.print(YY);
  Serial.print(" Z: ");Serial.print(YZ);Serial.println(""); 
  Serial.print("ACCE: ");
  Serial.print("X: ");Serial.print(CX);
  Serial.print(" Y: ");Serial.print(CY);
  Serial.print(" Z: ");Serial.print(CZ);Serial.println(""); 
  Serial.print("line: ");
  Serial.print("X: ");Serial.print(LX);
  Serial.print(" Y: ");Serial.print(LY);
  Serial.print(" Z: ");Serial.print(LZ);Serial.println(""); 
  Serial.print("GRAV: ");
  Serial.print("X: ");Serial.print(RX);
  Serial.print(" Y: ");Serial.print(RY);
  Serial.print(" Z: ");Serial.print(RZ);Serial.println(""); 
  Serial.print(temp);Serial.println(" C");Serial.println("");
  */
  
  //from master
  if (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    if (inChar == '\n') {stringComplete = true;}
  }
  if (stringComplete) {
    if (inputString.substring(0,5) == "$MSRR"){
      int pipeLoc = (inputString.indexOf('|'));
      int check = inputString.substring(6,pipeLoc).toInt();
      int measured = (inputString.length()-pipeLoc-3);
      if (measured == check){
        holder = inputString.substring(pipeLoc+1,inputString.length());
      }
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  //gps
  char c = GPS.read();
  //if (GPSECHO)
  //  if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())){ return; }}
      // this also sets the newNMEAreceived() flag to false
      // we can fail to parse a sentence in which case we should just wait for another
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  if (millis() - timer > 200) {
    sensors_event_t event; 
    bno.getEvent(&event);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> mags = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); 
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> line = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    String hT = "%T"; int8_t temp = bno.getTemp();
    String hEx = "%EX"; float EX = euler.x(); 
    String hEy = "%EY"; float EY = euler.y(); 
    String hEz = "%EZ"; float EZ = euler.z(); 
    String hMx = "%MX"; float MX = mags.x(); 
    String hMy = "%MY"; float MY = mags.y(); 
    String hMz = "%MZ"; float MZ = mags.z();
    String hYx = "%YX"; float YX = gyro.x(); 
    String hYy = "%YY"; float YY = gyro.y(); 
    String hYz = "%YZ"; float YZ = gyro.z();
    String hCx = "%CX"; float CX = accel.x(); 
    String hCy = "%CY"; float CY = accel.y(); 
    String hCz = "%CZ"; float CZ = accel.z();
    String hLx = "%LX"; float LX = line.x(); 
    String hLy = "%LY"; float LY = line.y(); 
    String hLz = "%LZ"; float LZ = line.z();
    String hRx = "%RX"; float RX = grav.x(); 
    String hRy = "%RY"; float RY = grav.y(); 
    String hRz = "%RZ"; float RZ = grav.z();

    String header = "$MSRS";
    String hAx = "%AX"; float AX = (event.orientation.x); //accel
    String AXs = String(AX,4);
    String hAy = "%AY"; float AY = (event.orientation.y); //accel
    String AYs = String(AY,4);
    String hAz = "%AZ"; float AZ = (event.orientation.z); //accel 
    String AZs = String(AZ,4); 
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
    String GLs = String(GL,6);
    String hGg = "%GG"; double GG = GPS.longitude;//long
    String GGs = String(GG,6);
    String hGp = "%GP"; float GP = GPS.speed;//speed
    String hGt = "%GT"; float GT = GPS.altitude;//alt
    String hGn = "%GN"; float GN = GPS.angle;//ang
    String hGa = "%GA"; int GA = GPS.satellites;//sats
    char Glat = GPS.lat; char Glon = GPS.lon;
    
    String BNO=
      hEx+EX+','+
      hEy+EY+','+
      hEz+EZ+','+
      hMx+MX+','+
      hMy+MY+','+
      hMz+MZ+','+
      hYx+YX+','+
      hYy+YY+','+
      hYz+YZ+','+
      hCx+CX+','+
      hCy+CY+','+
      hCz+CZ+','+
      hLx+LX+','+
      hLy+LY+','+
      hLz+LZ+','+
      hRx+RX+','+
      hRy+RY+','+
      hRz+RZ+','+
      hT+temp+','+
      hAx+AXs+','+
      hAy+AYs+','+
      hAz+AZs+',';
    String GPS=
      hGh+GH+','+
      hGm+GM+','+
      hGs+GS+','+
      hGi+GI+','+
      hGd+GD+','+
      hGo+GO+','+
      hGy+GY+','+
      hGf+GF+','+
      hGq+GQ+','+
      hGl+GLs+Glat+','+
      hGg+GGs+Glon+','+
      hGp+GP+','+
      hGt+GT+','+
      hGn+GN+','+
      hGa+GA+',';

    String TXpre = BNO+GPS+holder;
     
    int sLen = TXpre.length();
    String TX = header+','+sLen+'|'+TXpre;
    Serial.println(TX);
    //Serial.println(holder);
        
    timer = millis(); // reset the timer

    //sd card
    File dataFile = SD.open("datalog3.txt", FILE_WRITE);
    dataFile.println(TX);
    dataFile.close();
        
    /*
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
    */
  }
}
