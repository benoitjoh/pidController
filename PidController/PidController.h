// **************************************************************************
//                                                       Johannes Benoit 2017
// **************************************************************************

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"


class PidController
{
    public:
        // methods
        void initialize(int kp, int ki, int kd, int max_pwr, int max_d, int max_i, int intervall_millis);
        void regulate(int desired, int actual);
 
        // parameters
        unsigned int ocrDelta;
        bool pcc_is_on;
        byte signal_pin;
        byte output_pin;
        int max_power;
 
    protected:
    private:


};

#endif // PIDCONTROLLER_H
