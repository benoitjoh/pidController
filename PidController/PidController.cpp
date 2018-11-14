#include "PidController.h"
#include "Arduino.h"


#define INCREASE_FACTOR 100

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

PidController::PidController(int max_e, int max_eSum, int intervall_millis, int max_pwr)
{
    this->max_e = max_e; 
    this->max_eSum = max_eSum; 
    this->intervall_millis = intervall_millis; 
    this->max_pwr = max_pwr; 
   
    pccPowerLastCheckMillis = 0; //time, the pwr was last adjusted.
    
    eLast = 0;       // delta (actual - desired) at last measurement // for the d-summand 
    eSum = 0;        // integration of all deltas in the past // for the I-summand
    resultPowerExact = 0; // calculation result must be finer than pccPower

}

void PidController::set_parameters(int kp, int ki, int kd)
{
    this->kp = kp * INCREASE_FACTOR; 
    this->ki = ki * INCREASE_FACTOR; 
    this->kd = kd * INCREASE_FACTOR; 
}

int PidController::regulate(int desired, int actual)
{
	
    long e = 0;
    long d = 0;
    long y = 0;
    
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
            
            // integration for the i part
            eSum = constrain(eSum + e, -max_eSum, max_eSum); 
            
            // save for next call difference at last measurement, used for D - part
            eLast = e; 
             
            // PID formula 
            y = - ( kp * e + ki * eSum + kd * d );
            
 
            resultPowerExact += (float)y / INCREASE_FACTOR / 10;
            

            // map the float to integer parameter pccPower and limit it to 0 .. max range 
            resultPowerExact = constrain(resultPowerExact, 0, max_pwr);
            
#ifdef DEBUG_SPEEDCONTROL
           Serial.print("SPC:\t" + lFill(String(actual), 6, " ") + 
                            "\t" + lFill(String(e), 6, " ") + 
                            "\t" + lFill(String(eSum), 6, " ") + 
                            "\t" + lFill(String(d), 6, " ") + 
                            "\t" + lFill(String(y / INCREASE_FACTOR), 8, " ") + 
                            "\t" + lFill(String(int(resultPowerExact)), 6, " ") + "\n");
#endif     

        }
    }
    return int(resultPowerExact);   	
    

}


