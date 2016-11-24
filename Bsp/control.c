#include "control.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "data.h"


#define MODE1

Control_t ctrl;
/*
typedef struct
  {
    float32_t A0;          < The derived gain, A0 = Kp + Ki + Kd . 
    float32_t A1;          < The derived gain, A1 = -Kp - 2Kd. 
    float32_t A2;          < The derived gain, A2 = Kd . 
    float32_t state[3];    < The state array of length 3. 
    float32_t Kp;          < The proportional gain.
    float32_t Ki;          < The integral gain. 
    float32_t Kd;          < The derivative gain.
  } arm_pid_instance_f32;

*/
arm_pid_instance_f32 sp_pitch;
arm_pid_instance_f32 sp_roll;
arm_pid_instance_f32 sp_yaw;

arm_pid_instance_f32 pp_pitch;
arm_pid_instance_f32 pp_roll;
arm_pid_instance_f32 pp_yaw;



void MotorInit()
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);//motor1  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//motor2  
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);//motor3
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);//motor4    
}
void throttle(uint16_t data)
{
  data=CLMAP(data,1,1200);
  ctrl.power=data;
  MT1=data;
  MT2=data;
  MT3=data;
  MT4=data;
}
void PidUpate()
{
  arm_pid_init_f32(&sp_pitch,1);
  arm_pid_init_f32(&pp_pitch,1);
  arm_pid_init_f32(&sp_roll,1);
  arm_pid_init_f32(&pp_roll,1);
  arm_pid_init_f32(&sp_yaw,1);
  arm_pid_init_f32(&pp_yaw,1);
}
void ControlUpdate()
{
  float gx,gy,pitch,roll;
  float out1,out2;
  gx=Data.pSensor->gyro[0];
  gy=Data.pSensor->gyro[1];
  roll=Data.euler[0];
  pitch=Data.euler[1];
  out1=arm_pid_f32(&pp_pitch,0-roll);
  out2=arm_pid_f32(&pp_roll,0-pitch);
  
#ifdef MODE1
  {
    out1=arm_pid_f32(&sp_roll,0-gx*57.3); //pitch
    out2=arm_pid_f32(&sp_pitch,0-gy*57.3);  //roll
  }
#else
  {
    out1=arm_pid_f32(&sp_roll,out1-gx); //pitch
    out2=arm_pid_f32(&sp_pitch,out2-gy);  //roll
  }
  out1=CLMAP(out1,0,1200);
  out2=CLMAP(out2,0,1200);
#endif
//  MT1=(uint16_t)(ctrl.power+out1+out2);
//  MT2=(uint16_t)(ctrl.power-out1-out2);
//  MT3=(uint16_t)(ctrl.power-out1+out2);
//  MT4=(uint16_t)(ctrl.power+out1+out2);
//  MT1=(uint16_t)(ctrl.power+out1);
//  MT2=(uint16_t)(ctrl.power-out1);
//  MT3=(uint16_t)(ctrl.power-out1);
//  MT4=(uint16_t)(ctrl.power+out1);
  MT1=(uint16_t)(ctrl.power-out2);
  MT2=(uint16_t)(ctrl.power-out2);
  MT3=(uint16_t)(ctrl.power+out2);
  MT4=(uint16_t)(ctrl.power+out2);
}




