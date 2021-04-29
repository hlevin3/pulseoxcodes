/* Pulse Oximeter Analysis Testing Program
 * 
 * This is an example code adapted from Prof Daresh Desai's 
 * maxim analysis routines previously used with the max3102.
 * 
 * Red and IR LED's are controlled by the Arduino. 
 * 
*/
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// scaling factor for R (used as argument to o2anal)
// SET THE CORRECT VALUE OF THIS BASED ON YOUR CALIBRATION
#define ALPHA 1.2

// The extra DELAY TIME (in ms) between calls to sample the LED's
// This controls the frequency of sampling.  If the delay is too 
// short, the analysis cannot span the time of several heartbeats.
// if it is too long, we would not get good results for the peaks.

#define SAMPLE_DELAY 20

// This code was developed on PlatformIO using VSCODE.
// the main program therefore is .cpp rather than .ino
// to change to .ino just rename 
// (and optionally remove the Aeduino.h include line below)

// Place the files o2anal.cpp and o2anal.h in the folder with the
// arduino version of this source code.

#include <Arduino.h> // FOR THE SAKE OF C++

// Include the header file for the analysis code in o2anal.cpp
#include "o2anal.h"

LiquidCrystal_I2C lcd(0x20,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/* CONSTANTS DECLARING THE PINS USED -- ADJUST FOR YOUR CIRCUIT */
const byte Sensor = A0;  // analog input from OPT101
const byte RedLed = 2;   // pin for red LED
const byte NIRLed = 3;   // pin for near infrared LED

/* ***** SAMPLING PROCEDURE ****** 
The functon sample controls the measurment of light intensities.
It starts assuming that the LED's are turned off.
In then measures the intensities for background, red, IR,
Before each measurement, there is a possible to allow for the
photodiode to stabilize.  (This is mostly needed when the photodiode 
needs to recover from being saturated).

RELATED PARAMETERS: 
  THOLD = time in microseconds to wait before analog read of LED's
  THODLINIT = time micorseconds to wait before analof read of background.

NOTE: an analogRead() takes about 104 microseconds!!
*/

#define THOLD 100
#define THOLDINIT 100

void sample(int *background,int *red, int *nir){
    // the three arguments are pointers to result variables.
    // assume both LED's are off
    delayMicroseconds(THOLDINIT);
    *background = analogRead(Sensor);
    digitalWrite(RedLed,HIGH);
    delayMicroseconds(THOLD);
    *red = analogRead(Sensor);
    digitalWrite(RedLed,LOW);
    digitalWrite(NIRLed,HIGH);
    delayMicroseconds(THOLD);
    *nir=analogRead(Sensor);
    digitalWrite(NIRLed,LOW);
} 


// Global Variables
float heartrate, oxygen; // current results (these could be locals in loop)
int bufstatus;  // integer return value from the anaylsis routine

//========== SETUP ============
void setup() {
    // Initialize serial communication line
    Serial.begin(115200);  // NOTE: 115200 BAUD for Serial Printer
    
    // Output Pins for LED's
    pinMode(RedLed, OUTPUT);
    pinMode(NIRLed, OUTPUT);
    // initial state is off:
    digitalWrite(RedLed,LOW);
    digitalWrite(NIRLed,LOW); 

    // do 1 analogRead to init ADC
    analogRead(Sensor);


    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.noAutoscroll();
}

// *** DEBUGGING RELATED VARIABLES == 
// printed to help track what is happening

// For debugging print some lines of output from sensors
const int dumplines = 0;  // set to 0 for no dump
int linesdone=0;
long analysiscalls=0; // number of calls to anal since last result
long lastresulttime=0; // time of last result (ms)
// END OF DEBUGGING VARIABLES

void loop() {
  int irDatum, redDatum, backDatum; // values from sample
  long now ; //stores current millis  FOR DEBUGGING 

  /* Acquire the next a red/ir sample */
  sample(&backDatum,&redDatum, &irDatum);

/* DEBUGGING: PRINT OUR READINGS
 * this will print dumplines times and then stop
 */
//    if (linesdone++ <= dumplines){
//        Serial.print(millis());  // time in ms
//        Serial.print("\t");
//        Serial.print(backDatum);
//        Serial.print("\t");
//        Serial.print(redDatum);
//        Serial.print("\t");
//        Serial.println(irDatum);
//    }
/* END DEBUGGING */

  // Note that redDataum and irDataum are 16 bits here and 
  // are declared as 32 bit ints in the function definition interface currently
  // From Maxim the data was 18 bits with MSB in bit 17. \
  // Repair this later

  // a patch to avoid sending negative values to analysis
  if (redDatum<backDatum) redDatum=backDatum;
  if (irDatum<backDatum) irDatum=backDatum;

  analysiscalls +=1; // USED FOR DEBUG OUTPUT BELOW

  if (maybe_get_pulseOxy(&heartrate, &oxygen, &bufstatus, 
                          redDatum-backDatum, irDatum-backDatum, ALPHA)){
    // Here there has been a new estimate from the analysis
    lcd.clear();
    Serial.print(" sa02:");
    lcd.print(" sa02: ");
    Serial.print(oxygen);
    lcd.print(oxygen);
    Serial.print(" hr:");
    lcd.setCursor(0,1);
    lcd.print(" hr: ");
    Serial.print(heartrate);  
    lcd.print(heartrate);
    // printout number of calls to analysis since last result
    // and the average frequency of the analysis calls.
    now=millis();
    Serial.print("  {");Serial.print(analysiscalls);Serial.print(" calls, ");
    Serial.print((1000.0*analysiscalls)/(now-lastresulttime));
    Serial.println("Hz}");
    lastresulttime=now;  // reset time of last result to now
    analysiscalls=0; // reset call count
  } 

  
  // DELAY to decrease sampling rate to around 25 Hz like Maxim:
   delay(SAMPLE_DELAY);  // guessing all the above is under 5ms (without prints)
   
}
