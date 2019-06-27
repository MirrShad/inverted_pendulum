#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_ftm.h"
#include "ctr_motor.h"


void ctr_motor_init(void)
{
    gpio_init(PTB23, GPO, HIGH);
    gpio_init(PTB21, GPO, HIGH);

    ftm_pwm_init(FTM0, FTM_CH0, 100, 0);//占空比为0/1000
}


void ctr_motor(int16 pwm)
{
    if(pwm>0)
    {
        gpio_init(PTB23, GPO, LOW);
        gpio_init(PTB21, GPO, HIGH);

        ftm_pwm_duty(FTM0, FTM_CH0, pwm);
    }
    if(pwm<0)
    {
        gpio_init(PTB23, GPO, HIGH);
        gpio_init(PTB21, GPO, LOW);

        ftm_pwm_duty(FTM0, FTM_CH0, -pwm);
    }
    if(pwm==0)
    {
        gpio_init(PTB23, GPO, HIGH);
        gpio_init(PTB21, GPO, HIGH);

        ftm_pwm_duty(FTM0, FTM_CH0, 0);
    }
}
