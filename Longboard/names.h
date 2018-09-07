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
uint16_t setpoint = 0;
int error = 0;
int rpmConvert = 0;

//for stringBuilding
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

String header = "$MSRR";
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
