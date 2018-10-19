// Diagnose program to display reads of all ad-inputs on display

#include <LiquidCrystal_SR2W.h>
LiquidCrystal_SR2W lcd(8, 7, POSITIVE);

#include<AnalogKbd.h>
AnalogKbd kbd(0, 5, 40, 400);
byte kbdValue = 255; //the value that is read from keyboard



#include<RpmMeter.h>
#define PIN_SIGNAL   2  // pin for the signal of sensor
#define SIG_PER_TURN 1  // signals during one turn
#define SAMPLES_COUNT 1 // take a sum over signals 

// --- ports for input/output of phase cut control  
#include <PhaseCutCtrl.h>
#define PIN_IN_ACZERO_SIGNAL   3  // pin for AC zeropoint detection (Interrupt source, so can only be 2 or 3) 
#define PIN_PCC_OUT_A          4  // pin for pulse cut modulation output
#define PCC_POWER_MAX 999 

#include <PidController.h>
//int kp, int ki, int kd, int max_i, int max_d, int intervall_millis, int max_pwr
#define PID_KP 2
#define PID_KI 0
#define PID_KD 0
#define PID_MAX_E 50
#define PID_MAX_E_SUM 100
#define PID_INTERVALL_MILLIS 200 

PidController pid(PID_KP, PID_KI, PID_KD, PID_MAX_E, PID_MAX_E_SUM, PID_INTERVALL_MILLIS, PCC_POWER_MAX);

int pccPower = 0; // value for the phase cut modulation control 0 ... 999; zero is OFF
int pccPowerLast = 0;
int desiredRpm = 0;
int actualRpm = 0;

#define PIN_ANALOG_TACHO 1

void setup()
{
  RPM.initialize(PIN_SIGNAL, SIG_PER_TURN, SAMPLES_COUNT);
  PCCtrl.initialize(PIN_IN_ACZERO_SIGNAL, PIN_PCC_OUT_A, PCC_POWER_MAX);
  lcd.begin(16,2);               // initialize the lcd
  lcd.home();                   // go home
  lcd.setBacklight(1);
  lcd.print("Adjustment for");
  lcd.setCursor(0,1);
  lcd.print("pid controller");
  lcd.noCursor();
  delay(1200);
  lcd.clear();
  Serial.begin(9600);

}

String leftFill(String a, byte len, String letter)
{
    // fills a string with letters from left side that the resulting length is reached
    while (a.length() < len)
    {
        a = letter + a;
    }
    return a;
}

#define STEP 50


void loop()
{
    kbdValue = kbd.read();

    switch  (kbdValue)
    {
        case  2:
           desiredRpm++;
           break;
        case  0:
           desiredRpm += STEP;
           break;
        case  3:
           desiredRpm--;
           break;
        case  1:
           desiredRpm -= STEP;
           break;
        default:
           break;
        
    }
    kbdValue = 255;
    
    int adValue = analogRead(PIN_ANALOG_TACHO);
    actualRpm = RPM.getRpm();

    pccPower = pid.regulate(desiredRpm, actualRpm);

    if (pccPower != pccPowerLast)
    {
        PCCtrl.set_pcc(pccPower);
    }
    pccPowerLast = pccPower; 
    

    lcd.home();
    lcd.print("ad" + leftFill(String(adValue), 4, "0") + " set" + leftFill(String(desiredRpm), 6, " ") + " ");
    lcd.setCursor(0,1);
    lcd.print("P " + leftFill(String(pccPower), 4, "0")+ " act" + leftFill(String(actualRpm), 6, " "));
     delay(25);
}



