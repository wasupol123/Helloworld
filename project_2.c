/*Base register adddress header file*/
#include "stm32l1xx.h"
/*Library related header files*/
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_tim.h"

#include "dwt_delay.h"

/*already implemented */
void SystemClock_Config(void);
void OW_WriteBit(uint8_t d);
uint8_t OW_ReadBit(void);
void DS1820_GPIO_Configure(void);
uint8_t DS1820_ResetPulse(void);

/*haven't been implemented yet!*/
void OW_Master(void);
void OW_Slave(void);
void OW_WriteByte(uint8_t data);
uint16_t OW_ReadByte(void);

uint32_t CheckDigit(uint32_t);
void segment(uint32_t);
uint32_t CharToUint32_t(char number);
void ltc4727_GPIO_Config(void);

uint16_t temp;
float temp_cal;
uint8_t status;
char LOG[10] = "";
uint32_t seg[4] = {
	| LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 };
uint32_t digit[4] = {LL_GPIO_PIN_0 , LL_GPIO_PIN_1 , LL_GPIO_PIN_2 , LL_GPIO_PIN_3};

int main()
{
	SystemClock_Config();
	DWT_Init();
	DS1820_GPIO_Configure();
	ltc4727_GPIO_Config();
	
	while(1)
	{
		//Send reset pulse
		DS1820_ResetPulse();
			
		//Send 'Skip Rom (0xCC)' command
		OW_WriteByte(0xCC);
		
		//Send 'Temp Convert (0x44)' command
		OW_WriteByte(0x44);
		
		//Delay at least 200ms (typical conversion time)
		LL_mDelay(200);
		
		//Send reset pulse
		DS1820_ResetPulse();
		
		//Send 'Skip Rom (0xCC)' command
		OW_WriteByte(0xCC);
		
		//Send 'Read Scractpad (0xBE)' command
		OW_WriteByte(0xBE);
		
		//Read byte 1 (Temperature data in LSB)
		//Read byte 2 (Temperature data in MSB)
		temp = OW_ReadByte();
		
		//Convert to readable floating point temperature
		temp_cal = (temp*1.0) / 16.0;

		segment((uint32_t)temp_cal);
		
		for(int i=0;i<4;i++){
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);//Write 0 to GPIOC
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);//Reser all segment
			LL_GPIO_SetOutputPin(GPIOC, digit[i]);
			LL_GPIO_SetOutputPin(GPIOB, seg[i]);
		}
	}
}

void OW_WriteBit(uint8_t d)
{
	if(d == 1) //Write 1
	{
		OW_Master(); //uC occupies wire bus
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
		DWT_Delay(1);
		OW_Slave(); //uC releases wire bus
		DWT_Delay(60);
	}
	else //Write 0
	{
		OW_Master(); //uC occupies wire bus
		DWT_Delay(60);
		OW_Slave(); //uC releases wire bus
	}
}

uint8_t OW_ReadBit(void)
{
	OW_Master();
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	DWT_Delay(2);
	OW_Slave();
	
	return LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6);	
}


void DS1820_GPIO_Configure(void)
{
	LL_GPIO_InitTypeDef ds1820_io;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	ds1820_io.Mode = LL_GPIO_MODE_OUTPUT;
	ds1820_io.Pin = LL_GPIO_PIN_6;
	ds1820_io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ds1820_io.Pull = LL_GPIO_PULL_NO;
	ds1820_io.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOB, &ds1820_io);
}

uint8_t DS1820_ResetPulse(void)
{	
	OW_Master();
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	DWT_Delay(480);
	OW_Slave();
	DWT_Delay(80);
	
	DWT_Delay(75);
	if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6) == 0)
	{
		/*
		LOG[0] = 'E';
		LOG[1] = 'R';
		LOG[2] = 'R';
		LOG[3] = 'O';
		LOG[4] = 'R';
		*/
		return 0;
	}
	else
	{
		/*
		LOG[0] = 'O';
		LOG[1] = 'K';
		*/
		return 1;
	}
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}

void OW_Master(void){
	LL_GPIO_InitTypeDef ds1820_io;
	
	ds1820_io.Mode = LL_GPIO_MODE_OUTPUT;
	ds1820_io.Pin = LL_GPIO_PIN_6;
	ds1820_io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ds1820_io.Pull = LL_GPIO_PULL_NO;
	ds1820_io.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOB, &ds1820_io);
}

void OW_Slave(void){
	LL_GPIO_InitTypeDef ds1820_io;
	
	ds1820_io.Mode = LL_GPIO_MODE_INPUT;
	ds1820_io.Pin = LL_GPIO_PIN_6;
	ds1820_io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ds1820_io.Pull = LL_GPIO_PULL_NO;
	ds1820_io.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOB, &ds1820_io);
}

void OW_WriteByte(uint8_t data){
	uint8_t i,temp;
	
	//OW_Master();		//The data line is configured as output
	for(i = 0; i < 8; i++)
	{
	    //temp = data&0x01;
        //if(temp)              
            OW_WriteBit(data&0x01);	//Write 1
        //else                  
            //OW_WriteBit(0);		//Write 0 	
        data >>= 1;						//There are 8 places in total, starting from the low position and then moving to the right	
	}
	//OW_Slave();
}

uint16_t OW_ReadByte(void){
	uint16_t i,th,tl,temp;
	th = 0;
	tl = 0;
  for(i = 0; i < 8; i++)
  {
		tl |= OW_ReadBit() << i;		//Read low first
		DWT_Delay(60);
	}
	for(i = 0; i < 8; i++)
  {
		th |= OW_ReadBit() << i;		//Read low first
		DWT_Delay(60);
	}
	temp = (th<<8) | tl;
	return temp;
}

uint32_t CheckDigit(uint32_t number)
{
    int i;
    int digit;
    char seg[4];
    for(i=0;number!=0;i++)
    {
        number = number/10;
    }
    return i;
}

void segment(uint32_t number)
{
    uint32_t digit;
    char numberForShow[4];
    digit = CheckDigit(number);
    switch(digit)
    {
    case 1:
        numberForShow[0] = '0';
        numberForShow[1] = '0';
        numberForShow[2] = '0';
        numberForShow[3] = number+'0';
        break;
    case 2:
        numberForShow[0] = '0';
        numberForShow[1] = '0';
        numberForShow[2] = (number/10)+'0';
        numberForShow[3] = (number%10)+'0';
        break;
    case 3:
        numberForShow[0] = '0';
        numberForShow[1] = (number/100)+'0';
        numberForShow[2] = (number%100/10)+'0';
        numberForShow[3] = (number%100%10)+'0';
        break;
    case 4:
        numberForShow[0] = (number/1000)+'0';
        numberForShow[1] = (number%1000/100)+'0';
        numberForShow[2] = (number%1000%100/10)+'0';
        numberForShow[3] = (number%1000%100%10)+'0';
        break;
    }
		
		for(int i=0;i<4;i++)
		{
			seg[i] = CharToUint32_t(numberForShow[i]);
		}
}

uint32_t CharToUint32_t(char number)
{
    switch(number)
    {
        case '0':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
        case '1':
            return LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
        case '2':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15;
        case '3':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15;
        case '4':
            return LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
        case '5':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
        case '6':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
        case '7':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
        case '8':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
        case '9':
            return LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
    }
}

void ltc4727_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef ltc4727_init;
	
	//config ltc4727js
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
		
	ltc4727_init.Mode = LL_GPIO_MODE_OUTPUT;
	ltc4727_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ltc4727_init.Pull = LL_GPIO_PULL_NO;
	ltc4727_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	ltc4727_init.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB, &ltc4727_init);
		
	ltc4727_init.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC, &ltc4727_init);
}