
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
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
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
void GPIO_Config(void);

uint32_t CheckDigit(uint32_t);
void segment(uint32_t);
uint32_t CharToUint32_t(char number);
void ltc4727_GPIO_Config(void);

void ConvertDigitToArrayR_L(int);
void ConvertTOSeg(void);
void ShowSeg(void);
void show_a(void);
uint8_t dp = LL_GPIO_PIN_5; //DP>> PA5	
uint16_t temp;
float temp_cal;
uint8_t status;
char LOG[10] = "";
uint32_t save[4] = {0};
uint32_t seg1[10] = {LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14, //0
										 LL_GPIO_PIN_10 | LL_GPIO_PIN_11, //1
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_15 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12, //2
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_15 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12, //3
										 LL_GPIO_PIN_14 | LL_GPIO_PIN_10 | LL_GPIO_PIN_15 | LL_GPIO_PIN_11, //4
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12, //5
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, //6
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11, //7
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, //8
										 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 //9
									
	};


uint32_t digit[4] = {LL_GPIO_PIN_0 , LL_GPIO_PIN_1 , LL_GPIO_PIN_2 , LL_GPIO_PIN_3};
int catmsg = 1234;


#define E_O6					(uint16_t)1318
#define C_06					(uint16_t)1046
#define D_06					(uint16_t)1174
#define F_06					(uint16_t)1396
#define A_06					(uint16_t)1760
#define B_06					(uint16_t)1975
#define MUTE					(uint16_t) 1


/*for 10ms update event*/
#define TIMx_PSC	2


/*Macro function for ARR calculation*/
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))

void SystemClock_Config(void);
void TIM_BASE_Config(uint16_t);
void TIM_OC_GPIO_Config(void);
void TIM_OC_Config(uint16_t);
void TIM_BASE_DurationConfig(void);


int state = 0;
int sheetnote[] = {C_06,C_06,D_06,B_06,C_06,D_06,MUTE,E_O6,E_O6,F_06,E_O6,D_06,MUTE,C_06,D_06,C_06,B_06,C_06};
int i=0;

int main()
{
	DWT_Init();
	DS1820_GPIO_Configure();
	ltc4727_GPIO_Config();
	
	SystemClock_Config();
	
	while(1)
	{
		//Send reset pulse
		DS1820_ResetPulse();
			
		//Send 'Skip Rom (0xCC)' command
		OW_WriteByte(0xCC);
		
		//Send 'Temp Convert (0x44)' command
		OW_WriteByte(0x44);
		
		//Delay at least 200ms (typical conversion time)
		//LL_mDelay(200);
		
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

		//segment((uint32_t)temp_cal);
		ConvertDigitToArrayR_L((int)temp_cal*100);
		
		for(int i=0;i<4;i++)
		{
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);//Write 0 to GPIOC
			LL_GPIO_ResetOutputPin(GPIOB, seg1[8]);//Reser all segment
			
			LL_GPIO_SetOutputPin(GPIOB, seg1[save[i]]);
			LL_GPIO_SetOutputPin(GPIOC, digit[i]);
			
			if(i < 3)
				LL_mDelay(2);
		}
		

		//ConvertDigitToArrayR_L((int)temp_cal*100);
		//ConvertDigitToArrayR_L(catmsg);
		//show_a();
		//ShowSeg();
		
		if (temp_cal > 20.00)
			
		{
		TIM_OC_Config(ARR_CALCULATE(E_O6));
		}
			
		else 
			{
			TIM_OC_Config(ARR_CALCULATE(A_06));
		  }
	}
}

void TIM_BASE_DurationConfig(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 100 - 1;
	timbase_initstructure.Prescaler =  32000 - 1;
	LL_TIM_Init(TIM2, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM2); 
	LL_TIM_ClearFlag_UPDATE(TIM2); //Force clear update flag
}

void TIM_BASE_Config(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	LL_TIM_Init(TIM4, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM4); 
}


void TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void TIM_OC_Config(uint16_t note)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_OC_GPIO_Config();
	TIM_BASE_Config(note);
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 2;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}

void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
	}
}

void OW_WriteBit(uint8_t d)
{
	if(d == 1) //Write 1
	{
		OW_Master(); //uC occupies wire bus
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
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
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
	DWT_Delay(2);
	OW_Slave();
	
	return LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7);	
}


void DS1820_GPIO_Configure(void)
{
	LL_GPIO_InitTypeDef ds1820_io;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	ds1820_io.Mode = LL_GPIO_MODE_OUTPUT;
	ds1820_io.Pin = LL_GPIO_PIN_7;
	ds1820_io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ds1820_io.Pull = LL_GPIO_PULL_NO;
	ds1820_io.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOB, &ds1820_io);
}

uint8_t DS1820_ResetPulse(void)
{	
	OW_Master();
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
	DWT_Delay(480);
	OW_Slave();
	DWT_Delay(80);
	DWT_Delay(64);
	
	if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7) == 0)
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


void ConvertDigitToArrayR_L(int tempConv)
{
	for(int i = 0 ; i < 4 ; ++i) //convert num to digit array L->R
		{
			if(i == 0)
			{
				save[i] = tempConv /1000;			
			}
			else if(i == 1)
			{
				save[i] = tempConv/100 %10;
			}
			else if(i == 2)
			{
				save[i] = (tempConv %100)/10;
			}
			else if(i ==3)
			{
				save[i] = tempConv %10;
			}
		}
		catmsg = tempConv;
}
void show_a(void)
{
			for(int i=0;i<4;i++)
	  {
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);//Write 0 to GPIOC
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);//Reser all segment
			LL_GPIO_SetOutputPin(GPIOC, digit[i]);
			LL_GPIO_SetOutputPin(GPIOB, seg1[save[i]]);
		};
}

/*
void ConvertTOSeg(void)
{
	for(int i = 0 ; i < 4 ; ++i) //convert digit to leg 7-seg
		{
			switch(save[i]){
				case 0 : save[i] = seg1[0];
								 break;
			  case 1 : save[i] = seg1[1];
								 break;
				case 2 : save[i] = seg1[2];
								 break;
				case 3 : save[i] = seg1[3];
								 break;
				case 4 : save[i] = seg1[4];
								 break;
				case 5 : save[i] = seg1[5];
								 break;
				case 6 : save[i] = seg1[6];
								 break;
				case 7 : save[i] = seg1[7];
								 break;
				case 8 : save[i] = seg1[8];
								 break;
				case 9 : save[i] = seg1[9];
								 break;
			}
		}
}
*/

void ShowSeg(void)
{
	for(int i = 0 ; i < 4 ; ++i) //show 7-seg
		{
			
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3); // Write 0 to GPIOC port
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); // Reset all segment (PB2,PB10-PB15)
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
				LL_GPIO_SetOutputPin(GPIOB, save[i]);
        LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				if(i==1)
					LL_GPIO_SetOutputPin(GPIOA,dp);
				if(i<3)
				{
					LL_mDelay(1);
				}
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
	ds1820_io.Pin = LL_GPIO_PIN_7;
	ds1820_io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ds1820_io.Pull = LL_GPIO_PULL_NO;
	ds1820_io.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOB, &ds1820_io);
}

void OW_Slave(void){
	LL_GPIO_InitTypeDef ds1820_io;
	
	ds1820_io.Mode = LL_GPIO_MODE_INPUT;
	ds1820_io.Pin = LL_GPIO_PIN_7;
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

/*
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
*/

/*
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
*/

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
