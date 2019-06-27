#include "common.h"
#include "pid.h"

float Kp_ang=60;
float Ki_ang=0;
float Kd_ang=70;

/*******************************************************************************
                             Ang_PID(位置式)
*******************************************************************************/
int16 Ang_PID(float measured, float target)
{
    int16 result;
    float error;
    static float error_last=0;//上次偏差
    static float error_integration=0;//偏差积分

    error=measured-target;//计算偏差
    error_integration+=error;//对偏差积分
    result=(int16)(  Kp_ang*error+Ki_ang*error_integration+Kd_ang*(error-error_last)  );
    error_last=error;//更新此值

    return result;
}
