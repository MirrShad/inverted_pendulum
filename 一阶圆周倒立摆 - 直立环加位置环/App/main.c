#include "common.h"
#include "include.h"

#define ANGLE_BALANCE 880

uint16 angle;
int64  encoder_counter=0;

int16  signed_pwm_ang;

int16  signed_pwm_pos_last=0;
int16  signed_pwm_pos_now;
int16  signed_pwm_pos_delta;
int16  signed_pwm_pos_used;

int16  signed_pwm_all;

void main(void)
{
    adc_init(ADC1_SE10);                        //ADC初始化
    ctr_motor_init();                           //电机控制初始化
    ftm_quad_init(FTM2);                        //编码器 FTM2

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
    encoder_counter += ftm_quad_get(FTM2);
    ftm_quad_clean(FTM2);

    signed_pwm_ang = Ang_PID(angle, ANGLE_BALANCE);

    static uint8 ct = 0;
    ct++;
    if (ct == 1)
    {
        signed_pwm_pos_now = Pos_PID(encoder_counter, 0);
        signed_pwm_pos_delta = signed_pwm_pos_now-signed_pwm_pos_last;
        signed_pwm_pos_used = signed_pwm_pos_last+(signed_pwm_pos_delta*1/10);
    }
    if (ct == 2){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*2/10);}
    if (ct == 3){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*3/10);}
    if (ct == 4){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*4/10);}
    if (ct == 5){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*5/10);}
    if (ct == 6){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*6/10);}
    if (ct == 7){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*7/10);}
    if (ct == 8){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*8/10);}
    if (ct == 9){signed_pwm_pos_used=signed_pwm_pos_last+(signed_pwm_pos_delta*9/10);}
    if (ct == 10)//1不分频 2二分频 3三分频 以此类推
    {
        ct = 0;

        signed_pwm_pos_used = signed_pwm_pos_last+(signed_pwm_pos_delta*10/10);
        signed_pwm_pos_last = signed_pwm_pos_now;
    }

    signed_pwm_all = -signed_pwm_ang + signed_pwm_pos_used;

    if (signed_pwm_all > 1000)  signed_pwm_all = 1000;
    if (signed_pwm_all < -1000) signed_pwm_all = -1000;

    if ((angle>(ANGLE_BALANCE-100)) && (angle<(ANGLE_BALANCE+100)))
    {
        ctr_motor(signed_pwm_all);
    }
    else
    {
        ctr_motor(0);
    }
}
