// **************************************************************************
// Implementation for a pid controller
// 
// adjustment units:
// constants for PID / PD controller 
// #define PID_KP 0  // factor for p summand
// #define PID_KI 0  // factor for i summand
// #define PID_KD 0  // factor for d summand
// #define PID_MAX_E 50  // maximum absulute difference (actual - desired)
// #define PID_MAX_E_SUM 100 // maximum absulute integral for e
// #define PID_INTERVALL_MILLIS 200 // period between two calculations
// 
// 20W    fan: INIT_PWR 60, KP 7,  KD 80,  LIN 30,  CHANGE_MILLIS 500
// 500W motor: INIT_PWR 0,  KP 7,  KD 20,  LIN 80,  CHANGE_MILLIS 200   // fast
// 500W motor: INIT_PWR 0,  KP 1,  KD 20,  LIN 120,  CHANGE_MILLIS 200 // calm

//
// regulate(): 
// calculates the pccPower (0 .. max) value depending from target value and actual value of speed
// the class uses a exact float value for the power value, and returns an int() from that value
//
// (for theorie see: http://rn-wissen.de/wiki/index.php/Regelungstechnik)
// 
//  e is the difference between target and actual
// 
// PID controller 
//    esum = esum + e
//    y = Kp * e + Ki * Ta * esum + Kd * (e - eLast)/Ta
//    eLast = e
//
//                                                       Johannes Benoit 2017
// **************************************************************************

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"


class PidController
{
    public:
        // methods
        PidController(int max_e, int max_eSum, int intervall_millis, int max_pwr);
        void set_parameters(byte kp, byte ki, byte kd);
        int regulate(int desired, int actual);
        void checkOverload();
 
    private:
        // parameters
        int kp; 
        int ki; 
        int kd;
        long max_pwr;
        int max_e;
        int max_eSum; 
        int intervall_millis; 
        unsigned long pccPowerLastCheckMillis; //time, the pwr was last adjusted.
		long eLast = 0;       // delta (actual - desired) at last measurement // for the d-summand 
		long eSum = 0;        // integration of all deltas in the past // for the I-summand
		long resultPowerExact = 0; // calculation result must be finer than pccPower

        
    protected:


};

#endif // PIDCONTROLLER_H
