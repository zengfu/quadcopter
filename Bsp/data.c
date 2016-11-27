#include "stm32f4xx_hal.h"
#include "data.h"
#include "mpu9250.h"
#include "arm_math.h"
#include "imu.h"
#include "string.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


static void Quat2Euler();
static void Euler2Quat();

static SensorData_t SensorData;

Data_t Data;

static short gyro_offset[3]={0};

void DataInit()
{
  float roll,pitch,yaw;
  short gyro[3];
  for(int i=0;i<200;i++)
  {
    MPU9250_Get3AxisGyroRawData(gyro);
    gyro_offset[0]+=gyro[0];
    gyro_offset[1]+=gyro[1];
    gyro_offset[2]+=gyro[2];
    HAL_Delay(1);
    
  }
  gyro_offset[0]/=200;
  gyro_offset[1]/=200;
  gyro_offset[2]/=200;
  //TODO: the part need low filter to reduce tolerance
  memset((uint8_t*)&SensorData,0,sizeof(SensorData));
  UpdateSensorData();  
  Data.pSensor=&SensorData;
  
  roll=atan2(Data.pSensor->accel[1],Data.pSensor->accel[2]);  //
  //pitch=(asin(CLMAP(Data.pSensor->accel[1] / 9.8,-1.0,1.0)));
  pitch=-atan2(Data.pSensor->accel[0],Data.pSensor->accel[2]);
  //yaw=atan2(my*arm_cos_f32(roll)+mx*arm_sin_f32(roll)*arm_sin_f32(pitch)-mz*arm_sin_f32(roll)*arm_cos_f32(pitch)
  //          ,mx*arm_cos_f32(pitch)-mz*arm_sin_f32(pitch));
  yaw=0;
  Data.euler[0]=roll*57.3;
  Data.euler[1]=pitch*57.3;
  Data.euler[2]=yaw*57.3;
  Euler2Quat();
}
void UpdateData()
{
  UpdateSensorData();//get the new data;
  IMUupdate();//caculate the pitch and roll
  Quat2Euler();//
}

void Quat2Euler()
{
  float q0,q1,q2,q3;
  float pitch,roll,yaw;
  
  q0=Data.q[0];
  q1=Data.q[1];
  q2=Data.q[2];
  q3=Data.q[3];
  
  
  yaw=Data.euler[2];
   
  //roll=asin(CLMAP(2 * (q2 * q3 + q0 * q1) , -1.0f , 1.0f));
  //pitch=-atan2(2 * (q1 * q3 - q0* q2) , 1- 2 * (q2 * q2+ q1 * q1));
  //yaw=atan2(my*arm_cos_f32(roll)+mx*arm_sin_f32(roll)*arm_sin_f32(pitch)-mz*arm_sin_f32(roll)*arm_cos_f32(pitch),mx*arm_cos_f32(pitch)-mz*arm_sin_f32(pitch))*57.3;
  //yaw = -(0.9 * (-yaw + gz*0.002*57.3) + 5.73* atan2(mx*cos(roll) + my*sin(roll)*sin(pitch) + mz*sin(roll)*cos(pitch), my*cos(pitch) - mz*sin(pitch)));
  //yaw=atan2(2 * (q0 * q2 + q3 * q1) , 1 - 2 * (q1 * q1 + q2 * q2))*57.3; 
  //yaw=atan2(mx,my)*57.3;
  roll=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  pitch=asin(CLMAP(2*(q0*q2-q1*q3),-1.0f,1.0f));
  yaw=atan2(2*(q0*q3+q1*q2),1-2*(q3*q3+q2*q2));
  Data.euler[0] = roll*57.3; 
  Data.euler[1]  = pitch*57.3;
  //Data.euler[2] =0.9*(yaw1+gz*57.3*0.002)+0.1*yaw;
  //Data.euler[2] = atan2(2 * (q0 * q2 + q3 * q1) , 1 - 2 * (q1 * q1 + q2 * q2))*57.3; 
  //Data.euler[2] =0.9*(yaw-gz*57.3*0.002)+0.1*yaw;
  Data.euler[2]=yaw*57.3;
}  

void Euler2Quat()
{
  
  float recipNorm;
  float fCosHRoll = arm_cos_f32(Data.euler[0]/57.3 * .5f);
  float fSinHRoll = arm_sin_f32(Data.euler[0]/57.3 * .5f);
  float fCosHPitch = arm_cos_f32(Data.euler[1]/57.3 * .5f);
  float fSinHPitch = arm_sin_f32(Data.euler[1]/57.3 * .5f);
  float fCosHYaw = arm_cos_f32(Data.euler[2]/57.3 * .5f);
  float fSinHYaw = arm_sin_f32(Data.euler[2]/57.3* .5f);
  float q0,q1,q2,q3;

  /// Cartesian coordinate System
  q0 = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
  q1 = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
  q2 = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
  q3 = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;

//  q0 = fCosHRoll * fCosHPitch * fCosHYaw - fSinHRoll * fSinHPitch * fSinHYaw;
//  q1 = fCosHRoll * fSinHPitch * fCosHYaw - fSinHRoll * fCosHPitch * fSinHYaw;
//  q2 = fSinHRoll * fCosHPitch * fCosHYaw + fCosHRoll * fSinHPitch * fSinHYaw;
//  q3 = fCosHRoll * fCosHPitch * fSinHYaw + fSinHRoll * fSinHPitch * fCosHYaw;
        
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  Data.q[0]=q0;
  Data.q[1]=q1;
  Data.q[2]=q2;
  Data.q[3]=q3;      
}

void UpdateSensorData()
{
  short accel[3],gyro[3],mag[3];
  MPU9250_Get9AxisRawData(accel, gyro, mag);
  SensorData.accel[0]=accel[0]/32768.0*2.0*9.8;//m2/s
  SensorData.accel[1]=accel[1]/32768.0*2.0*9.8;
  SensorData.accel[2]=accel[2]/32768.0*2.0*9.8;
  SensorData.gyro[0]=(gyro[0]-gyro_offset[0])/32768.0*2000/57.32;//radio
  SensorData.gyro[1]=(gyro[1]-gyro_offset[1])/32768.0*2000/57.32;
  SensorData.gyro[2]=(gyro[2]-gyro_offset[2])/32768.0*2000/57.32;
  SensorData.mag[0]=(float)mag[0];
  SensorData.mag[1]=(float)mag[1];
  SensorData.mag[2]=(float)mag[2];
}

extern UART_HandleTypeDef huart1;
void transport()
{
  uint8_t data[13]={0};
  uint16_t cnt=0;
  float roll,pitch,yaw;
  roll=Data.euler[0];
  pitch=Data.euler[1];
  yaw=Data.euler[2];
  //data[cnt++]=0xaa;
//  roll=5.5;
//  pitch=5.4;
//  yaw=5.3;
  data[cnt++]=BYTE0(roll);
  data[cnt++]=BYTE1(roll);
  data[cnt++]=BYTE2(roll);
  data[cnt++]=BYTE3(roll);
  data[cnt++]=BYTE0(pitch);
  data[cnt++]=BYTE1(pitch);
  data[cnt++]=BYTE2(pitch);
  data[cnt++]=BYTE3(pitch);
  data[cnt++]=BYTE0(yaw);
  data[cnt++]=BYTE1(yaw);
  data[cnt++]=BYTE2(yaw);
  data[cnt++]=BYTE3(yaw);
  //data[cnt++]=0x0;
  //data[cnt++]=0x00;
//  data[cnt++]=0x0;
//  data[cnt++]=0x0;
//  data[cnt++]=0x00;
//  data[cnt++]=0x00;
  printf("roll:%f,pitch:%f,yaw:%f\n",roll,pitch,yaw);
  //HAL_UART_Transmit(&huart1,data,12,100);
  //HAL_UART_Transmit_DMA(&huart1,data,12);
}




