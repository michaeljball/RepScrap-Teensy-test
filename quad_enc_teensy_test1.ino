/***************************************************************************************
*  quad_enc_teensy_test1.ino   01-12-2015   unix_guru at hotmail.com   @unix_guru on twitter
*  http://arduino-pi.blogspot.com
*
*  This sketch allows you to run two salvaged printer carriages for X/Y axis using their 
*  linear encoder strips for tracking. 
*  This example uses the Arduino PID Library found at:
*  https://github.com/br3ttb/Arduino-PID-Library/archive/master.zip
*
*  Flex Timer Modules are used for managing the two Quadrature Encoders as per 
*  Trudy Benjamin's wonderful library at: 
*  https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x
*  
*****************************************************************************************/

#include <i2c_t3.h>  // wire for Teensy 3.1 as per https://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PID_v1.h> 
#include "LiquidCrystal.h"

// Connect LiquidCrystal display via i2c, default address #0x20 (A0-A2 not jumpered)
LiquidCrystal lcd(0);


#include <Time.h>  //  https://github.com/PaulStoffregen/Time    Teensy RTC

time_t getTeensy3Time() { return Teensy3Clock.get(); }


#define frontstop = 100            // Right most encoder boundary
#define backstop = 3600            // Left most encoder boundary

#include "QuadDecode_def.h"        // Include the FlexTimer QuadDecoder library
#include <stdbool.h>

#define GUI_UPDATE_TIME	 500000	  // 500 mSec update
IntervalTimer serialTimer;	  // How often to update LCD display
void timerInt(void);	          // Main interval timing loop interrupt

#define PID_UPDATE_TIME	 5000	  // 5 mSec update
IntervalTimer pidTimer;	  // How often to update LCD display
void pidTimerInt(void);	          // Main interval timing loop interrupt



// Variables from CMM program
volatile int32_t rtX=0, rtY=0;	                // Realtime values of X,Y
volatile int32_t ltX=0, ltY=0;	                // Latched values of X,Y
volatile bool zero_XaxisPos=0, zero_rtY=0;      // Zero values of X,Y
volatile bool doOutput=false;	                // Run output routine
volatile bool mode = 0;                         // Random or linear test
int linearStep = 10;                             // How far to travel between steps

QuadDecode<1> xPosn;	// Template using FTM1
QuadDecode<2> yPosn;	// Template using FTM2


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *XaxisMotor = AFMS.getMotor(1);
Adafruit_DCMotor *YaxisMotor = AFMS.getMotor(2);


double Spd, XaxisSpd,  YaxisSpd;        // Carriage speed from 0-255

/*working variables for PID routines*/
// Tuning parameters
float KpX=0.6,  KpY=1;              //Initial Proportional Gain 
float KiX=40, KiY=100;             //Initial Integral Gain 
float KdX=0.02,  KdY=0.5;              //Initial Differential Gain 

double XaxisSetpoint=0, YaxisSetpoint=0;      // Taget position for carriage
double XaxisPos=0, YaxisPos=0;	          // Realtime values of X,Y

// Instantiate X and Y axis PID controls
PID XaxisPID(&XaxisPos, &XaxisSpd, &XaxisSetpoint, KpX, KiX, KdX, DIRECT); 
PID YaxisPID(&YaxisPos, &YaxisSpd, &YaxisSetpoint, KpY, KiY, KdY, DIRECT); 
const int sampleRate = 1; 

int led = 13;
bool acqd = LOW;

char buffer[32];        // Used for formatting numbers to display


// ================================== Variables related to Command Processing ==============================================================
        
char Command = 's';
int Parameter = 0;

char inData[50];                                           // Buffer for the incoming data
char *inParse[50];                                         // Buffer for the parsed data chunks


String inString = "";                                      // Storage for data as string
int chindex = 0;
boolean stringComplete = false;




void setup() {

// set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);         

  Serial.begin (115200);
  Serial.print("Linear Encoder Test");
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);

  pinMode(led, OUTPUT); 

  if (timeStatus()!= timeSet) {
  Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  // set up the LCD's number of rows and columns: 
  lcd.begin(20, 4);

  pinMode(led, OUTPUT);  
  lcd.print("Linear Encoder Test");
  lcd.setCursor(0, 1);
  lcd.print("01-12-2015");

  xPosn.setup();	                        // Start Quad Decode position count
  yPosn.setup();	                        // Start Quad Decode position count

  serialTimer.begin(timerInt,GUI_UPDATE_TIME);	// run timerInt routine every X period

  xPosn.start();	                        // Start Quad Decode position count
  yPosn.start();	                        // Start Quad Decode position count

  AFMS.begin();                                 // Set up Motors
  
  delay(1000);                                  // 1 second delay to show dislay
  randomSeed(analogRead(0));                    // Used to select random setpoints for testing
 
  XaxisPID.SetMode(AUTOMATIC);                  // Turn on the PID loop 
  XaxisPID.SetSampleTime(sampleRate);           // Sets the sample rate 
  XaxisPID.SetOutputLimits(-255,255);           // Set max speed for DC motors

  YaxisPID.SetMode(AUTOMATIC);                  // Turn on the PID loop 
  YaxisPID.SetSampleTime(sampleRate);           // Sets the sample rate 
  YaxisPID.SetOutputLimits(-255,255);           // Set max speed for DC motors

}

void loop() {          // NOTE:  ONLY WORKING X-AXIS for this sketch
  
    rtX=xPosn.calcPosn();   rtY=yPosn.calcPosn();   // Get current Xaxis position
    XaxisPos=rtX;   YaxisPos=rtY;	            // Realtime values of X,Y
    
    // For diagnostic  comment out later
      Serial.print("Tgt "); Serial.print(XaxisSetpoint,2);
      Serial.print(" Spd "); Serial.print(XaxisSpd,2);
      Serial.print(" Pos "); Serial.print(rtX);Serial.print("\r\n");
    
    if(XaxisPos == XaxisSetpoint) {              // Only X-axis atm
      XaxisMotor->run(RELEASE); 
      // If X-axis has reached it's target - get new target
      if(mode==0) {                              // Default is random
        XaxisSetpoint =  random(200,1000);       // Keep target within bounds of Endpoints
      } else XaxisSetpoint += linearStep;
      
      acqd = !acqd;                              // Toggle LED
      digitalWrite(led, acqd);
      delay(500);
    }  

    if (doOutput) updateDisplay(); 
  
    // Manage axis positioning
    XaxisPID.Compute();   YaxisPID.Compute(); 
    if(XaxisSetpoint < XaxisPos) XaxisMotor->run(BACKWARD);  // Determine direction of travel
    else  XaxisMotor->run(FORWARD);      
    XaxisMotor->setSpeed(abs(XaxisSpd));              // Apply PID speed to motor

    if(YaxisSetpoint < YaxisPos) YaxisMotor->run(BACKWARD);  // Determine direction of travel
    else  YaxisMotor->run(FORWARD);      
    YaxisMotor->setSpeed(abs(YaxisSpd));              // Apply PID speed to motor

  SerialEvent();                                            // Grab characters from Serial
  
  // =======================   if serial data available, process it ========================================================================
  if (stringComplete)                   // if there's any serial available, read it:
  {
    ParseSerialData();                  // Parse the recieved data
    inString = "";                      // Reset inString to empty   
    stringComplete = false;             // Reset the system for further input of data
  }  

}


void updateDisplay() {            // update LCD readout
    if (doOutput){
      doOutput=false;
      
      lcd.setCursor(0, 0);
      lcd.print("XTarget "); lcd.print(XaxisSetpoint,2);lcd.print("       ");
      lcd.setCursor(0, 1);
      sprintf(buffer, "%6d", rtX);

      lcd.print("XPosition "); lcd.print(buffer); 
      
      lcd.setCursor(0, 2);
      digitalClockDisplay(); 
           
    }
}

void digitalClockDisplay() {
  // digital clock display of the time
  lcd.print(hour());
  printDigits(minute());
  printDigits(second());
  lcd.print(" ");
  lcd.print(month());
  lcd.print(" ");
  lcd.print(day());
  lcd.print(" ");
  lcd.print(year());
}


void printDigits(int digits){
// utility function for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if(digits < 10) lcd.print('0');
  lcd.print(digits);
}


void timerInt(void){                                // Do this action every Interval timeout.
    doOutput=true;
}


void ParseSerialData()
{

  char *p = inData;                // The data to be parsed
  char *str;                       // Temp store for each data chunk
  int count = 0;                   // Id ref for each chunk
    
  while ((str = strtok_r(p, ",", &p)) != NULL)    // Loop through the data and seperate it into chunks at each "," delimeter
  { 
    inParse[count] = str;      // Add chunk to array  
    count++;      
  }

  if(count > 1)     // If the data has two values then..  
  {
    // Define value 1 as a Command identifier
    char *Command = inParse[0];
    // Define value 2 as a Parameter value
    char *Parameter = inParse[1];

  Serial.print("CMD,"); Serial.print(Command); Serial.print(","); Serial.print(Parameter); Serial.print(","); Serial.println("0");
    
    // Call the relevant identified Commandtion  
    switch(*Command)
    {
      case 'p':                                                         // Set 'Proportional'
        KpX = atof(Parameter);       
        Serial.print("KpX = "); Serial.print(KpX); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("KpX = "); lcd.print(KpX); lcd.print("    ");
        break;

      case 'i':                                                         // Set "Integral'
        KiX = atof(Parameter);       
        Serial.print("KiX = "); Serial.print(KiX); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("KiX = "); lcd.print(KiX); lcd.print("    ");
        break;

      case 'd':                                                         // Set 'Derivative'
        KdX = atof(Parameter);       
        Serial.print("KdX = "); Serial.print(KdX); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("KdX = "); lcd.print(KdX); lcd.print("    ");
        break;

      case 's':                                                         // Set MAX Speed
        Spd = atoi(Parameter);   
        XaxisPID.SetOutputLimits(0-Spd,Spd);           
    
        Serial.print("Max Spd = "); Serial.print(Spd); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("Spd = "); lcd.print(Spd); lcd.print("    ");
        break;

      case 'r':                                                         // Randomize Target
        mode = 0;   
        Serial.print("Mode = Random \n\r");
        lcd.setCursor(0, 3);
        lcd.print("Mode = Random         "); 
        break;

      case 'l':                                                         // Randomize Target
        mode = 1;   
        XaxisSetpoint = 100;         
        linearStep = atoi(Parameter);   
        Serial.print("Mode = Linear Step ="); Serial.print(linearStep); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("Linear Step = "); lcd.print(linearStep);
        break;


     }    
  }
  
}


void SerialEvent() 
{
  while (Serial.available() && stringComplete == false)    // Read while we have data
  {
    char inChar = Serial.read();             // Read a character
    inData[chindex] = inChar;                  // Store it in char array
    chindex++;                                 // Increment where to write next  
    inString += inChar;                      // Also add it to string storage just in case, not used yet :)
    
    if (inChar == '\n' || inChar == '\r')                      // Check for termination character
    {
      chindex = 0;
      stringComplete = true;
    }
  }
}


