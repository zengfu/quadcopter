#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

typedef struct
{
  float desired;     //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit
  float iLimitLow;    //< integral limit
  float dt;           //< delta-time dt
#ifdef PID_EX
  float power;
#endif
}PID_TypeDef;

typedef struct ControlDefine
{
  uint16_t power;
  float pitch;
  float roll;
  float yaw;
  uint8_t mode;
}xControl;


void pidSetDt(PID_TypeDef* pid, const float dt);
void pidSetKd(PID_TypeDef* pid, const float kd);
void pidSetKi(PID_TypeDef* pid, const float ki);
void pidSetKp(PID_TypeDef* pid, const float kp);
void pidSetDesired(PID_TypeDef* pid, const float desired);
void pidReset(PID_TypeDef* pid);
void pidSetIntegralLimitLow(PID_TypeDef* pid, const float limitLow);
void pidSetIntegralLimit(PID_TypeDef* pid, const float limit);
float pidUpdate(PID_TypeDef *pid,float measured,uint8_t careTime);
void pidInit(PID_TypeDef* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt);

#define MT1 htim4.Instance->CCR4
#define MT2 htim3.Instance->CCR3
#define MT3 htim5.Instance->CCR4
#define MT4 htim5.Instance->CCR1

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void throttle(uint16_t data);
void control_init();
void control_updata();




#endif