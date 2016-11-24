#ifndef _CONTROL_H
#define _CONTROL_H


#include "stm32f4xx_hal.h"
#include "arm_math.h"


extern arm_pid_instance_f32 sp;
extern arm_pid_instance_f32 pp;

typedef struct ControlDefine
{
  uint32_t power;
  uint32_t pitch;
  uint32_t roll;
  uint32_t yaw;
}xControl;

typedef xControl Control_t;



#define MT1 htim4.Instance->CCR4
#define MT2 htim3.Instance->CCR3
#define MT3 htim5.Instance->CCR4
#define MT4 htim5.Instance->CCR1

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern Control_t ctrl;

void MotorInit();
void throttle(uint16_t data);
void PidUpate();
void ControlUpdate();

extern arm_pid_instance_f32 sp_pitch;
extern arm_pid_instance_f32 sp_roll;
extern arm_pid_instance_f32 sp_yaw;

extern arm_pid_instance_f32 pp_pitch;
extern arm_pid_instance_f32 pp_roll;
extern arm_pid_instance_f32 pp_yaw;

#endif