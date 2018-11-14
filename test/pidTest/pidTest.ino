// Diagnose program to display reads of all ad-inputs on display

#include <LiquidCrystal_SR2W.h>
LiquidCrystal_SR2W lcd(8, 7, POSITIVE);

#define PIN_ANALOG_TACHO 0

#include<AnalogKbd.h>
AnalogKbd kbd(PIN_ANALOG_TACHO, 5, 40, 400);
byte kbdValue = 255; //the value that is read from keyboard

// speed measurement tool for the motor
//   note: class is preinstantiated as RPM
#include<RpmMeter.h>
#define PIN_SIGNAL   2  // pin for the signal of sensor
#define SIG_PER_TURN 1  // signals during one turn
#define SAMPLES_COUNT 1 // take a sum over signals 

// phase cut controller  
//   note: class is preinstantiated as PCCtrl
#include <PhaseCutCtrl.h>
#define PIN_IN_ACZERO_SIGNAL   3  // pin for AC zeropoint detection (Interrupt source, so can only be 2 or 3) 
#define PIN_PCC_OUT_A          4  // pin for pulse cut modulation output
#define PCC_POWER_MAX 999    // represents 100% power

// the speed regulator
#include <PidController.h>
// controller parameters that can be changed to adjust response of the system
byte PID_KP = 2;
byte PID_KI = 0;
byte PID_KD = 6;

// fixed parameters
#define PID_MAX_E 200
#define PID_MAX_E_SUM 300
#define PID_INTERVALL_MILLIS 200 

PidController pid(PID_MAX_E, PID_MAX_E_SUM, PID_INTERVALL_MILLIS, PCC_POWER_MAX);

int pccPower = 0; // value for the phase cut modulation control 0 ... 999; zero is OFF
int pccPowerLast = 0;
int desiredRpm = 400;
int actualRpm = 0;


void setup()
{
  pid.set_parameters(PID_KP, PID_KI, PID_KD);

  RPM.initialize(PIN_SIGNAL, SIG_PER_TURN, SAMPLES_COUNT);
  PCCtrl.initialize(PIN_IN_ACZERO_SIGNAL, PIN_PCC_OUT_A, PCC_POWER_MAX);
  lcd.begin(16,2);              // initialize the lcd
  lcd.home();                   // go home
  lcd.setBacklight(1);
  lcd.print("pid adjuster");
  lcd.noCursor();
  delay(900);
  lcd.clear();
  Serial.begin(9600);

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
           PID_KP++;
           break;
        case  128:
           PID_KP--;
           break;
        case  1:
           PID_KI++;
           break;
        case  129:
           PID_KI--;
           break;
        case  2:
           PID_KD++;
           break;
        case  130:
           PID_KD--;
           break;

        
        default:
           break;
        
    }
    kbdValue = 255;
    pid.set_parameters(PID_KP, PID_KI, PID_KD);
    
    
    int adValue = analogRead(PIN_ANALOG_TACHO);
    actualRpm = RPM.getRpm();
    
    pccPower = pid.regulate(desiredRpm, actualRpm);

    if (pccPower != pccPowerLast)
    {
        PCCtrl.set_pcc(pccPower);
    }
    pccPowerLast = pccPower; 
    

    lcd.home();
    lcd.print("kp" + leftFill(String(PID_KP), 2, " ") + "  ki" + leftFill(String(PID_KI), 2, " ") + "  kd" + leftFill(String(PID_KD), 2, " "));
    lcd.setCursor(0,1);
    lcd.print("P" + leftFill(String(pccPower), 3, "0")+ " D" + leftFill(String(desiredRpm), 4, " ")+ " A" + leftFill(String(actualRpm), 4, " "));
     delay(25);
}



