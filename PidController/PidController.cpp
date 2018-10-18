#include "PidController.h"
#include "Arduino.h"

void PidController::initialize(int kp, int ki, int kd, int max_pwr, int max_d, int max_i, int intervall_millis)
{
    output_pin = o_pin;
    signal_pin = s_pin;
    hz_factor = 500000 * SAMPLES_AMOUNT;
    netFreqCnt = 0;
    max_power = max_pwr;

    noInterrupts();
    pinMode(signal_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(signal_pin), ISR_acIsZero, RISING);

    pinMode(output_pin, OUTPUT);

    pcc_is_on = false;

    // set up timer interrupt for Timer1
    TCCR1A = 0;
    TCCR1B = 0;

    bitSet(TCCR1B, CS11);    // specify 8 als Prescaler for timer 1
    bitSet(TIMSK1, OCIE1A);  // activate Timer output compare Interrupt A at timer 1

    interrupts();             // activate all interrupts
    OCR1A = 0;


}

void PidController::regulate(int desired, int actual)
{
	
    return;

}

