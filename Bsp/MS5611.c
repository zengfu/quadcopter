#include "stm32f4xx_hal.h"
#include "ms5611.h"
#include <math.h>

#define ADDR_W 0xEE // Module address write mode
#define ADDR_R 0xEF // Module address read mode 


#define CMD_RESET 0x1E // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1 0x00 // ADC D1 conversion
#define CMD_ADC_D2 0x10 // ADC D2 conversion
#define CMD_ADC_256 0x00 // ADC OSR=256
#define CMD_ADC_512 0x02 // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2056
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD 0xA0 // Prom read command 

extern I2C_HandleTypeDef hi2c3;

//********************************************************
//! @brief send command using I2C hardware interface
//!
//! @return none
//********************************************************
static void i2c_send(uint8_t cmd)
{
  HAL_I2C_Master_Transmit(&hi2c3, ADDR_W, &cmd, 1, HAL_MAX_DELAY);
} 


//********************************************************
//! @brief send reset sequence
//!
//! @return none
//********************************************************
void cmd_reset(void)
{
 i2c_send(CMD_RESET); // send reset sequence
 HAL_Delay(3); 
}

//********************************************************
//! @brief preform adc conversion
//!
//! @return 24bit result
//********************************************************
static uint32_t cmd_adc(uint8_t cmd)
{
   uint8_t data[3];
   uint32_t temp=0;
   i2c_send(CMD_ADC_CONV+cmd); // send conversion command
   switch (cmd & 0x0f) // wait necessary conversion time
   {
   case CMD_ADC_256 : HAL_Delay(1); break;
   case CMD_ADC_512 : HAL_Delay(3); break;
   case CMD_ADC_1024: HAL_Delay(4); break;
   case CMD_ADC_2048: HAL_Delay(6); break;
   case CMD_ADC_4096: HAL_Delay(10); break;
   }
   i2c_send(CMD_ADC_READ);
   
   HAL_I2C_Master_Receive(&hi2c3, ADDR_R, data, 3, HAL_MAX_DELAY);
   temp=((uint32_t)data[0])*65536;
   temp=temp+((uint32_t)data[1])*256;
   temp=temp+(uint32_t)data[2];
   
   return temp;
} 

//********************************************************
//! @brief Read calibration coefficients
//!
//! @return coefficient
//********************************************************
static uint16_t cmd_prom(char coef_num)
{
 uint8_t data[2];
 uint16_t rC=0;

 i2c_send(CMD_PROM_RD+coef_num*2); // send PROM READ command
 HAL_I2C_Master_Receive(&hi2c3, ADDR_R, data, 2, HAL_MAX_DELAY);
 rC=(uint16_t)data[0]*256;
 rC=rC+data[1];

 return rC;
}
//********************************************************
//! @brief calculate the CRC code
//!
//! @return crc code
//********************************************************
static uint8_t crc4(uint16_t* n_prom)
{
   int cnt; // simple counter
   unsigned int n_rem; // crc reminder
   unsigned int crc_read; // original value of the crc
   unsigned char n_bit;
   n_rem = 0x00;
   crc_read=n_prom[7]; //save read CRC
   n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
   for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
   {// choose LSB or MSB
      if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
      else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
      for (n_bit = 8; n_bit > 0; n_bit--)
      {
          if (n_rem & (0x8000))
          {
              n_rem = (n_rem << 1) ^ 0x3000; 
          }
          else
          {
              n_rem = (n_rem << 1);
          }
      }
  }
  n_rem= (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
  n_prom[7]=crc_read; // restore the crc_read to its original place
  return (n_rem ^ 0x0);
} 
 
//void read_prom(I2C_HandleTypeDef * hi2c,uint8_t *a)
//{
//  uint8_t data=0xa6;
// 
//  HAL_I2C_Master_Transmit(hi2c, 0xee, &data, 1, HAL_MAX_DELAY);
//  HAL_I2C_Master_Receive(hi2c, 0xee, a, 2, HAL_MAX_DELAY);
//} 
void ReadTempPres(float* temp,float* pres)
{
  
  uint32_t D1=0; // ADC value of the pressure conversion
  uint32_t D2=0; // ADC value of the temperature conversion
  uint16_t C[8]; // calibration coefficients

  float dT; // difference between actual and measured temperature
  float OFF; // offset at actual temperature
  float SENS; // sensitivity at actual temperature
  int i;
  uint8_t n_crc; // crc value of the prom 
  for (i=0;i<8;i++){ C[i]=cmd_prom(i);} 
  n_crc=crc4(C); 
  D2=cmd_adc(CMD_ADC_D2+CMD_ADC_4096); // read D2
  D1=cmd_adc(CMD_ADC_D1+CMD_ADC_4096); // read D1
  // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
  dT=D2-C[5]*pow(2,8);
  OFF=C[2]*pow(2,16)+dT*C[4]/pow(2,7);
  SENS=C[1]*pow(2,15)+dT*C[3]/pow(2,8);

  *temp=(2000+(dT*C[6])/pow(2,23))/100;
  *pres=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15))/100;  
  
  
}


