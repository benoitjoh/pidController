#include "PidController.h"
#include "Arduino.h"

void PidController::initialize(int kp, int ki, int kd, int max_pwr, int max_i, int max_d, int intervall_millis)
{
    this->kp = kp; 
    this->ki = ki; 
    this->kd = kd; 
    this->max_pwr = max_pwr; 
    this->max_i = max_i; 
    this->max_d = max_d; 
    this->intervall_millis = intervall_millis; 
    
 
}

void PidController::regulate(int desired, int actual)
{
	
	
    return;

}

