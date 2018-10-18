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
        void initialize(int kp, int ki, int kd, int max_pwr, int max_i, int max_d, int intervall_millis);
        void regulate(int desired, int actual);
 
    private:
        // parameters
        int kp; 
        int ki; 
        int kd;
        int max_pwr;
        int max_d;
        int max_i; 
        int intervall_millis; 
        
    protected:


};

#endif // PIDCONTROLLER_H
