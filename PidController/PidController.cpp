#include "PidController.h"
#include "Arduino.h"

#define DEBUG_SPEEDCONTROL 1

#ifdef DEBUG_SPEEDCONTROL
String lFill(String a, byte len, String letter)
{
    while (a.length() < len)
    {
        a = letter + a;
    }
    return a;
}
#endif

PidController::PidController(int kp, int ki, int kd, int max_e, int max_eSum, int intervall_millis, int max_pwr)
{
    this->kp = kp; 
    this->ki = ki; 
    this->kd = kd; 
    this->max_e = max_e; 
    this->max_eSum = max_eSum; 
    this->intervall_millis = intervall_millis; 
    this->max_pwr = max_pwr; 
   
    pccPowerLastCheckMillis = 0; //time, the pwr was last adjusted.
    
    eLast = 0;       // delta (actual - desired) at last measurement // for the d-summand 
    eSum = 0;        // integration of all deltas in the past // for the I-summand
    resultPowerExact = 0; // calculation result must be finer than pccPower

}

int PidController::regulate(int desired, int actual)
{
	
    int e = 0;
    int d = 0;
    float y = 0.;
    
    if (desired == 0)
    {
        // turn off immidiately and thats it ...
        resultPowerExact = 0;
    }
    else
    {
        // ... otherwise calculate in definded intervalls
        if (millis() - pccPowerLastCheckMillis > intervall_millis )
        {
            // now its time for a adjustment
            pccPowerLastCheckMillis = millis(); // save time of last adjustment
            
            e = constrain(actual - desired, -max_e, max_e); // diff between target and actual speed
            d = e - eLast;
            eSum = eSum + d; 
              
            // PID formula 
            y = - ( kp * e + ki * intervall_millis * eSum + kd * d / intervall_millis );
            
 
            resultPowerExact += y;
            

            // map the float to integer parameter pccPower and limit it to 0 .. max range 
            resultPowerExact = constrain(resultPowerExact, 0, max_pwr);
            
#ifdef DEBUG_SPEEDCONTROL
           Serial.print("SPC:\t" + lFill(String(actual), 6, " ") + 
                            "\t" + lFill(String(e), 6, " ") + 
                            "\t" + lFill(String(eSum), 6, " ") + 
                            "\t" + lFill(String(d), 6, " ") + 
                            "\t" + lFill(String(y), 8, " ") + 
                            "\t" + lFill(String(int(resultPowerExact)), 6, " ") + "\n");
#endif     
            // save for next call
            eLast = e; // difference at last measurement, used for D - part

        }
    }
    return int(resultPowerExact);   	
    

}


