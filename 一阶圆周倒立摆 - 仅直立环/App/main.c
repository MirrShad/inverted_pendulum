#include "common.h"
#include "include.h"

#define ANGLE_BALANCE 880

uint16 angle;
int16  signed_pwm;

void main(void)
{
    adc_init(ADC1_SE10);                        //ADC初始化
    ctr_motor_init();                           //电机控制初始化

    void PIT0_ISR();                            //定时器中断函数配置
    pit_init_ms(PIT0, 10);                      //初始化 PIT0 定时时间 10ms
    set_vector_handler(PIT0_VECTORn, PIT0_ISR); //设置 PIT0 的中断服务函数为 PIT0_ISR
    enable_irq(PIT0_IRQn);                      //开中断

    while (1)
    {
        printf("%d\n", angle);
        DELAY_MS(50);
    }
}
/*******************************************************************************
                             定时器中断函数(10ms)
*******************************************************************************/
void PIT0_ISR(void)
{
    //清中断标志位
    PIT_Flag_Clear(PIT0);

    //功能代码部分
    angle = adc_once(ADC1_SE10, ADC_10bit);

    signed_pwm = Ang_PID(angle, ANGLE_BALANCE);
    if (signed_pwm > 1000)  signed_pwm = 1000;
    if (signed_pwm < -1000) signed_pwm = -1000;

    if ((angle>(ANGLE_BALANCE-50)) && (angle<(ANGLE_BALANCE+50)))
    {
        ctr_motor(-signed_pwm);
    }
    else
    {
        ctr_motor(0);
    }
}
