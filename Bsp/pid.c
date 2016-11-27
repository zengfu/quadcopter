#include "pid.h"
#include "data.h"
#define DEFAULT_PID_INTEGRATION_LIMIT  500

PID_TypeDef euler_pitch;
PID_TypeDef euler_roll;
PID_TypeDef gyro_pitch;
PID_TypeDef gyro_roll;

//#define MODE1
extern Data_t Data;

static void MotorInit();
static void control_output(float* out);

#define grollkp 10
#define grollki 0.01
#define grollkd 10

#define erollkp 0.05
#define erollki 0.0
#define erollkd 0.05

xControl ctrl;

static void MotorInit()
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);//motor1  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//motor2  
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);//motor3
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);//motor4   
  htim4.Instance->CNT=0;
  htim3.Instance->CNT=0;
  htim5.Instance->CNT=0;
}

void throttle(uint16_t data)
{
  
  //data=CLMAP(data,1,500);
  ctrl.power=data;
  MT1=data;
  MT2=data;
  MT3=data;
  MT4=data;
}

void control_init()
{
  MotorInit();
  pidInit(&euler_pitch,0,erollkp,erollkp,erollkp,0);
  pidInit(&euler_roll,0,erollkp,erollki,erollkd,0);
  pidInit(&gyro_roll,0,grollkp,grollki,grollkd,0);
  pidInit(&gyro_pitch,0,grollkp,grollkp,grollkp,0);
}
void control_updata()
{
  float out[2];
  out[0]=pidUpdate(&euler_roll,Data.euler[0],0);
  out[1]=pidUpdate(&euler_pitch,Data.euler[1],0);
  pidSetDesired(&gyro_roll,out[0]);
  pidSetDesired(&gyro_pitch,out[1]); 
  out[0]=pidUpdate(&gyro_roll,Data.pSensor->gyro[0],0);
  out[1]=pidUpdate(&gyro_pitch,Data.pSensor->gyro[1],0);
  control_output(out);
}
static void control_output(float* out)
{
  //ctrl.power
  out[1]=0;
  if(ctrl.power==0)
  {
    MT1=0;
    MT2=0;
    MT3=0;
    MT4=0;
  }
  else
  {
    MT1=ctrl.power+(uint16_t)(out[1]-out[0]);
    MT2=ctrl.power+(uint16_t)(out[1]+out[0]);
    MT3=ctrl.power+(uint16_t)(-out[1]+out[0]);
    MT4=ctrl.power+(uint16_t)(-out[1]-out[0]);
  }
  
}


void pidInit(PID_TypeDef* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
}


float pidUpdate(PID_TypeDef *pid,float measured,uint8_t careTime)
{
  float output;
  pid->error=pid->desired-measured;
  
  if(careTime)
    pid->integ+=pid->error*pid->dt;
  else
    pid->integ+=pid->error;
  
  if(pid->integ>pid->iLimit)
  {
    pid->integ=pid->iLimit;
  }
  else if(pid->integ<pid->iLimitLow)
  {
    pid->integ=pid->iLimitLow;
  }
  else
  {
  }
  if(careTime)
  {
    pid->deriv=(pid->error-pid->prevError)/pid->dt;
  }
  else
  {
    pid->deriv=pid->error-pid->prevError;
  }
  pid->outI=pid->integ*pid->ki;
  pid->outD=pid->deriv*pid->kd;
  pid->outP=pid->error*pid->kp;
  
  output=pid->outD+pid->outI+pid->outP;
  
  pid->prevError=pid->error;
  return output;
}
void pidSetIntegralLimit(PID_TypeDef* pid, const float limit)
{
    pid->iLimit = limit;
}


void pidSetIntegralLimitLow(PID_TypeDef* pid, const float limitLow)
{
    pid->iLimitLow = limitLow;
}

void pidReset(PID_TypeDef* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetDesired(PID_TypeDef* pid, const float desired)
{
  pid->desired = desired;
}

void pidSetKp(PID_TypeDef* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PID_TypeDef* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PID_TypeDef* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PID_TypeDef* pid, const float dt) 
{
    pid->dt = dt;
}