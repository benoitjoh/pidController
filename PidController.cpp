#include "PidController.h"
#include "Arduino.h"

// int value that represents the maximum power:
#define MAX_POWER 2048

#define MINIMUM_STOP_TIME 500

// time bewteen two regulation events in milliseconds
#define INTERVALL_REGULATE_MILLIS 255

// if desired rpm is lower than this value, the y differential is divided by two
#define LOW_RPM_LIMIT 550


// turn on debug:
//#define DEBUG_SPEEDCONTROL 1

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



PidController::PidController(int max_e, int max_eSum)
{
    this->max_e = max_e;
    this->max_eSum = max_eSum;

    pccPowerLastCheckMillis = 0; //time, the pwr was last adjusted.

    eLast = 0;       // delta (actual - desired) at last measurement // for the d-summand
    eSum = 0;        // integration of all deltas in the past // for the I-summand
    resultPower = 0; // calculation result must be finer than pccPower
#ifdef DEBUG_SPEEDCONTROL
    pinMode(13, OUTPUT);
#endif // DEBUG_SPEEDCONTROL

}

void PidController::checkOverload()
{
	// check if  actual rpm remains zero for 2sec when load is > 80%
	// if that occurs, turn off to 0% and wait 10secs

}


void PidController::set_parameters(byte kp, byte ki, byte kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PidController::enable_direction_management(byte pinDirSwitch)
{
    // enable direction management
    this->pinDirSwitch = pinDirSwitch;
    pinMode(pinDirSwitch, OUTPUT);
    actualDirection = 0;
    digitalWrite(pinDirSwitch, actualDirection);
}

void PidController::set_direction(byte desiredDirection)
{
    isDirChangeRequested = (actualDirection != desiredDirection);
#ifdef DEBUG_SPEEDCONTROL
    Serial.print("SPC: direction change reqested \n");
#endif

}
byte PidController::get_direction()
{
    return actualDirection;
}



int PidController::regulate(int desired, int actual)
{
    if (pinDirSwitch != 255)
    {
        // Direction managment is enabled

        // determine stop state (for directinon change)
        if (actual == 0)
        {
            if (isStoped == false)
            {
                isStoped = true;
                stoppedSinceMillis = millis();
#ifdef DEBUG_SPEEDCONTROL
                Serial.print("SPC: STOPPED\n");
#endif
           }


        }
        else
        {
            isStoped = false;
        }

        // handle direction change request

        if (isDirChangeRequested)
        {
            desired = 0;

            if (isStoped == true)
            {

            if (millis() - stoppedSinceMillis > MINIMUM_STOP_TIME)
                {
                    //switch direction
                    actualDirection = ~actualDirection & 1;
                    isDirChangeRequested = false;
                    digitalWrite(pinDirSwitch, actualDirection);
#ifdef DEBUG_SPEEDCONTROL
                    Serial.print("SPC: dir set to " + String(actualDirection) + " \n");
#endif

                }
            }
        }
    }


    int y = 0;

    if (desired == 0)
    {
        // turn off immidiately and thats it ...
        resultPower = 0;
    }
    else
    {
        // ... otherwise calculate in definded intervalls
        if (millis() - pccPowerLastCheckMillis > INTERVALL_REGULATE_MILLIS )
        {

#ifdef DEBUG_SPEEDCONTROL
           PORTB |=  B00100000; //set pin13 to HIGH for timemeasurement
#endif // DEBUG_SPEEDCONTROL

            // now its time for a adjustment (calculation costs about 10 mySeconds)

            pccPowerLastCheckMillis = millis(); // save time of last adjustment

            int e = actual - desired; // diff between target and actual speed

            // integration for the i part
            // eSum is the over all integration, myeSum is limited for the use in the formula
            eSum += e;
            int myeSum = constrain(eSum, -max_eSum, max_eSum);

            // differential for d part
            int d = e - eLast;

            // save for next call difference at last measurement, used for D - part
            eLast = e;

            e = constrain(e, -max_e, max_e); // limit for use in formula


            // PID formula
            // take half of the value, that the parameters kp ki and kd allow finer adjustment
            y = - ( kp * e + (ki * myeSum) + kd * d ) >> 1;

            // in very small rpm areas, make the steps also smaller
            if (desired < LOW_RPM_LIMIT)
            {
                y =  y >> 1;
            }

            resultPower += int(y) ;

            // map the long to integer parameter pccPower and limit it to 0 .. max range*256
            resultPower = constrain(resultPower, 0, MAX_POWER);

#ifdef DEBUG_SPEEDCONTROL
           PORTB &= ~B00100000; //set pin13 back to LOW for timemeasurement

           Serial.print("SPC:\t" + lFill(String(actual), 6, " ") +
                            "\t" + lFill(String(desired), 5, " ") +
                            "\t" + lFill(String(e), 5, " ") +
                            "\t" + lFill(String(d), 5, " ") +
                            //"\t" + lFill(String(eSum), 5, " ") +
                            //"\t" + lFill(String(myeSum), 5, " ") +

                            "\t" + lFill(String(y), 5, " ") +
                            "\t" + lFill(String(resultPower>>1), 4, "0") + "\n");
#endif

        }
    }
    return resultPower;


}


