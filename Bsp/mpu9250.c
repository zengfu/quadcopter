#include "stm32f4xx_hal.h"
#include "mpu9250.h"

extern SPI_HandleTypeDef hspi1;

static int16_t MPU9250_AK8963_ASA[3] = {0, 0, 0};


//////////////////////////////////////////////////////////////////////////
//init
void MPU9250_Init(void)
{
	uint8_t data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	//Lower level hardware Init
	
	//////////////////////////////////////////////////////////////////////////
	//MPU9250 Reset
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	HAL_Delay(100);
	//MPU9250 Set Clock Source
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	HAL_Delay(1);
	//MPU9250 Set Interrupt
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, ENABLE);
	HAL_Delay(1);
	//MPU9250 Set Sensors
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	HAL_Delay(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	HAL_Delay(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	HAL_Delay(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));
	HAL_Delay(1);
	//MPU9250 Set Accel DLPF
	data = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, data);
	HAL_Delay(1);
	//MPU9250 Set Gyro DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	HAL_Delay(1);
	//MPU9250 Set SPI Mode
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	HAL_Delay(1);
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	HAL_Delay(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	HAL_Delay(2);

	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	HAL_Delay(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	HAL_Delay(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (int16_t)((data - 128.0f) / 256.0f + 1.0f) ;
	MPU9250_AK8963_ASA[0] = (int16_t)(response[0]) + 128;
	MPU9250_AK8963_ASA[1] = (int16_t)(response[1]) + 128;
	MPU9250_AK8963_ASA[2] = (int16_t)(response[2]) + 128;
	HAL_Delay(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	HAL_Delay(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	HAL_Delay(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	HAL_Delay(1);
	//
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	HAL_Delay(1);

	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	HAL_Delay(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	HAL_Delay(100);
}

/* low driver*/
static void Chip_Select()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  //HAL_Delay(1);
  for(int i=3;i>0;i--)
  {
  }
}
static void Chip_DeSelect()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //HAL_Delay(1);
}
int MPU9250_SPIx_Write(uint8_t addr, uint8_t reg_addr, uint8_t data)
{
  Chip_Select();
  HAL_SPI_Transmit(&hspi1,&reg_addr,1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1,&data,1, HAL_MAX_DELAY);
  Chip_DeSelect();
  return 0;
}
int MPU9250_SPIx_Writes(uint8_t addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{

	Chip_Select();
	HAL_SPI_Transmit(&hspi1,&reg_addr,1, HAL_MAX_DELAY);
        HAL_SPI_Transmit(&hspi1,data,len, HAL_MAX_DELAY);
        Chip_DeSelect();
	return 0;
}
uint8_t MPU9250_SPIx_Read(uint8_t addr, uint8_t reg_addr)
{
	uint8_t data = 0;
	uint8_t raddr = 0;
        raddr=0x80|reg_addr;

	Chip_Select();
	HAL_SPI_Transmit(&hspi1,&raddr,1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1,&data,1, HAL_MAX_DELAY);
	Chip_DeSelect();
	return data;
}
int MPU9250_SPIx_Reads(uint8_t addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	
  uint8_t raddr;
  raddr=0x80|reg_addr;
  

  Chip_Select();
  HAL_SPI_Transmit(&hspi1,&raddr,1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1,data,len, HAL_MAX_DELAY);
  Chip_DeSelect();
  return 0;
}

int MPU9250_AK8963_SPIx_Read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data)
{
  uint8_t status = 0;
  uint32_t timeout = 0;

  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
  HAL_Delay(1);
  reg_addr = akm_addr | MPU9250_I2C_READ;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
  HAL_Delay(1);
  reg_addr = MPU9250_I2C_SLV4_EN;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
  HAL_Delay(1);

  do {
          if (timeout++ > 50){
                  return -2;
          }
          MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
          HAL_Delay(1);
  } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
  MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
  return 0;
}

int MPU9250_AK8963_SPIx_Reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{
  uint8_t index = 0;
  uint8_t status = 0;
  uint32_t timeout = 0;
  uint8_t tmp = 0;

  tmp = akm_addr | MPU9250_I2C_READ;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
  HAL_Delay(1);
  while(index < len){
          tmp = reg_addr + index;
          MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
          HAL_Delay(1);
          tmp = MPU9250_I2C_SLV4_EN;
          MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
          HAL_Delay(1);

          do {
                  if (timeout++ > 50){
                          return -2;
                  }
                  MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
                  HAL_Delay(2);
          } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
          MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
          HAL_Delay(1);
          index++;
  }
  return 0;
}
int MPU9250_AK8963_SPIx_Write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
  uint32_t timeout = 0;
  uint8_t status = 0;
  uint8_t tmp = 0;

  tmp = akm_addr;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
  HAL_Delay(1);
  tmp = reg_addr;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
  HAL_Delay(1);
  tmp = data;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
  HAL_Delay(1);
  tmp = MPU9250_I2C_SLV4_EN;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
  HAL_Delay(1);

  do {
          if (timeout++ > 50)
                  return -2;

          MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
          HAL_Delay(1);
  } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
  if (status & MPU9250_I2C_SLV4_NACK)
          return -3;
  return 0;
}

int MPU9250_AK8963_SPIx_Writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{
  uint32_t timeout = 0;
  uint8_t status = 0;
  uint8_t tmp = 0;
  uint8_t index = 0;

  tmp = akm_addr;
  MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
  HAL_Delay(1);

  while(index < len){
          tmp = reg_addr + index;
          MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
          HAL_Delay(1);
          MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
          HAL_Delay(1);
          tmp = MPU9250_I2C_SLV4_EN;
          MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
          HAL_Delay(1);

          do {
                  if (timeout++ > 50)
                          return -2;
                  MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
                  HAL_Delay(1);
          } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
          if (status & MPU9250_I2C_SLV4_NACK)
                  return -3;
          index++;
  }
  return 0;
}


//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag)
{
	uint8_t data[22];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
	
	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];

	if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[21] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

	//ned x,y,z
	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get6AxisRawData(short *accel, short * gyro)
{
	uint8_t data[14];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 14, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisAccelRawData(short * accel)
{
	uint8_t data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 6, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisGyroRawData(short * gyro)
{
	uint8_t data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_GYRO_XOUT_H, 6, data);

	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisMagnetRawData(short *mag)
{
	uint8_t data[8];

	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_EXT_SENS_DATA_00, 8, data);
	if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[7] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[2] << 8) | data[1];
	mag[1] = (data[4] << 8) | data[3];
	mag[2] = (data[6] << 8) | data[5];

	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_GetTemperatureRawData(long *temperature)
{
	uint8_t data[2];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_TEMP_OUT_H, 2, data);
	temperature[0] = (((int16_t)data[0]) << 8) | data[1];
}

/*******************/
void read_whoami(SPI_HandleTypeDef *hspi,uint8_t* data)
{
  //cs=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(5);
  //
  
  uint8_t addr=0xf5;//read who am i
  HAL_SPI_Transmit(hspi,&addr,1, HAL_MAX_DELAY);
  HAL_SPI_Receive(hspi,data,1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


void Get_Ms(unsigned long *timestamp)
{
  *timestamp=HAL_GetTick();
}
/*==============================================================================================*/
/*==============================================================================================*/