// Diagnose program to adjust parameters of pid controller


#include <LiquidCrystal_SR2W.h>
LiquidCrystal_SR2W lcd(8, 7, POSITIVE);

#define PIN_ANALOG_KEYBOARD 0

#include<AnalogKbd.h>
AnalogKbd kbd(PIN_ANALOG_KEYBOARD, 5, 40, 400);
byte kbdValue = 255; //the value that is read from keyboard

// speed measurement tool for the motor
//   note: class is preinstantiated as RPM
#include<RpmMeter.h>
#define PIN_SIGNAL   2  // pin for the signal of sensor
#define SIG_PER_TURN 1  // signals during one turn
#define SAMPLES_COUNT 1 // take a sum over signals 

// phase cut controller  
//   note: class is preinstantiated as PCCtrl
//   min power = 0; max power = 2048
#include <PhaseCutCtrl.h>
#define PIN_IN_ACZERO_SIGNAL   3  // pin for AC zeropoint detection (Interrupt source, so can only be 2 or 3) 
#define PIN_PCC_OUT_A          4  // pin for pulse cut modulation output

// ---- EEPROM support
#include "eeAny.h"
#define EE_OFFSET 128


// the speed regulator
#include <PidController.h>
// controller parameters that can be changed to adjust response of the system
//   min power = 0; max power = 2048



// keep the adjustable pid parameters in a struct
struct
{
    byte kp = 2;
    byte ki = 0;
    byte kd = 16;
} pidconf;


// fixed parameters
#define PID_MAX_E 200
#define PID_MAX_E_SUM 300

PidController pid(PID_MAX_E, PID_MAX_E_SUM);

int pccPower = 0; // value for the phase cut modulation control 0 ... 2048; zero is OFF
int pccPowerLast = -1;
int desiredRpm = 600;
int actualRpm = 0;
int dir = 0;

// --- use driver for latch register
#include <LatchControl.h>
#define PIN_LATCH_DATACLOCK  10           // shiftregister: clock, data and l(and data) signal
LatchControl latch(PIN_LATCH_DATACLOCK);

void setup()
{
  if (analogRead(PIN_ANALOG_KEYBOARD) < 20)
    {
       EEPROM_readAnything(EE_OFFSET, pidconf);
    }
  pid.set_parameters(pidconf.kp, pidconf.ki, pidconf.kd);
  pid.enable_direction_management(5);

  RPM.initialize(PIN_SIGNAL, SIG_PER_TURN, SAMPLES_COUNT);
  PCCtrl.initialize(PIN_IN_ACZERO_SIGNAL, PIN_PCC_OUT_A);
  lcd.begin(16,2);              // initialize the lcd
  lcd.home();                   // go home
  lcd.setBacklight(1);
  lcd.print("pid test program");
  lcd.noCursor();
  delay(900);
  lcd.clear();
  Serial.begin(9600);

  // PIN 11 is the out put enable for the latch
  latch.reset();
  delay(500);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  
//  for (int i=0; i<2049; i++)
//  {
//      PCCtrl.set_pcc(i);
//  }
  
}
  
String leftFill(String a, byte len, String letter)
{
    // fills a string with letters from left side that the resultstring length is reached
    while (a.length() < len)
    {
        a = letter + a;
    }
    return a;
}


void loop()
{
    kbdValue = kbd.read();

    switch  (kbdValue)
    {
        // change rpm
        case  3:
           desiredRpm += 100;
           break;
        case  4:
           desiredRpm -= 100;
           break;
        // adjust the regulators parameters
        case  0:
           pidconf.kp++;
           break;
        case  128:
           pidconf.kp--;
           break;
        case  1:
           pidconf.ki++;
           break;
        case  129:
           pidconf.ki--;
           break;
        case  2:
           pidconf.kd++;
           break;
        case  130:
           pidconf.kd--;
           break;
        case  131:
           dir = ~dir & 1;
           delay(900);
           pid.set_direction(dir);
           break;

        case  132:
           EEPROM_writeAnything(EE_OFFSET, pidconf);
           lcd.clear(); 
           lcd.print("stored to EEPROM.");
           delay(900);

           break;

        
        default:
           break;
        
    }
    kbdValue = 255;
    pid.set_parameters(pidconf.kp, pidconf.ki, pidconf.kd);
    
    
    actualRpm = RPM.getRpm();
    pccPower = pid.regulate(desiredRpm, actualRpm);

    if (pccPower != pccPowerLast)
    {
        PCCtrl.set_pcc(pccPower);
    }
    pccPowerLast = pccPower; 
    
    String dirName = "R";
    if (dir == 1)
    {
        dirName = "L";
    }
    lcd.home();
    lcd.print("kp" + leftFill(String(pidconf.kp), 2, " ") + "  ki" + leftFill(String(pidconf.ki), 2, " ") + "  kd" + leftFill(String(pidconf.kd), 2, " "));
    lcd.setCursor(0,1);
    lcd.print(leftFill(String(pccPower), 4, "0")+ " D" + leftFill(String(desiredRpm), 4, " ")+ " " + leftFill(String(actualRpm), 4, " ")+ dirName);

    // magic eye
    int d = constrain((actualRpm - desiredRpm) / 3, -3, 3) + 3;
    latch.setComplete(1 << d);
    
    delay(25);
}
