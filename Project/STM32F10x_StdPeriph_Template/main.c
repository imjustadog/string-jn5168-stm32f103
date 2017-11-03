/**
  ******************************************************************************
  * @file    TIM/InputCapture/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "project.h"
#include <stdio.h>
#include <math.h>

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_Input_Capture
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_ICInitTypeDef  TIM_ICInitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
USART_InitTypeDef  USART_InitStructure;
SPI_InitTypeDef   SPI_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void SPI_Configuration(void);
void ADC_Configuration(void);
void MEASUREMENT(int group);
void DELAY(__IO uint32_t nCount);
void init_timer(void);
void init_dma(void);
void dma_NVIC(void);

uint16_t Capture[6][30]={0};

volatile uint8_t state = 0;
volatile uint8_t timeout_flag = 0;
volatile uint8_t sending = 0;
volatile uint8_t send_flag = 0;
volatile uint8_t capture_flag = 0;

char mode='R';
uint32_t size=25;
float Frequency = 0;
float Frequency_last[6];
uint8_t online_count[6] = {0,0,0,0,0,0};
uint8_t limit_count = 5;

float Temperature = 0;
float cycleaverage[6] = {0};

unsigned char board_num1 = 0xAA;
unsigned char board_num2 = 0xAB;
uint8_t module_per_group = 1;

int STIMULATE_LOW[6] = {
	200,
	200,
	200,
	200,
	200,
	200};

int STIMULATE_HIGH[6] = {
	2200,
	2200,
	2200,
	2200,
	2200,
	2200};

	
float	A = 0.0014051f;
float B = 0.0002369f;
float C = 0.0000001019f;

void uart_zigbee_senddata(uint8_t *str);
void uart_485_senddata(uint8_t *str, uint8_t num);
UART_SendTypeDef UART_SendEnum  = SEND_NONE;

uint8_t volt_buf[4] = {'S', 0, 0, 'E'};

uint8_t data_buf[36] = {
	'S',
	0,0,//ID
	0,0,0,0,0, //CH1 频率高8位 频率低8位 温度高8位 温度低8位
	1,0,0,0,0, //CH2 频率高8位 频率低8位 温度高8位 温度低8位
	2,0,0,0,0, //CH3 频率高8位 频率低8位 温度高8位 温度低8位
	3,0,0,0,0, //CH4 频率高8位 频率低8位 温度高8位 温度低8位 
	4,0,0,0,0, //CH5 频率高8位 频率低8位 温度高8位 温度低8位 
	5,0,0,0,0, //CH6 频率高8位 频率低8位 温度高8位 温度低8位 
	0,0,       //电量高8位 电量低8位
  'E'};

uint8_t start_buf[36] = {
	'S',
	0,0,//ID
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	'A',
	0xff,
	0x02,
	0xff,
  'A'};

uint8_t reply_buf[36] = {
	'S',
	0,0,//ID
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0xff,
	0xff,
	0x00,
	0xff,
  'A'};

uint8_t adc_channel_map[6] = {
	ADC_Channel_10,
	ADC_Channel_11,
	ADC_Channel_12,
	ADC_Channel_13,
	ADC_Channel_0,
	ADC_Channel_1};

GPIO_TypeDef* ADR_GPIO[8] = {
	GPIOB, 
	GPIOB, 
  GPIOB, 
  GPIOB, 
  GPIOB, 
  GPIOC,
	GPIOC,
	GPIOC};

uint16_t ADR_PIN[8] = {
	ID_PIN_ADR1, 
  ID_PIN_ADR2, 
  ID_PIN_ADR3, 
  ID_PIN_ADR4, 
  ID_PIN_ADR5, 
  ID_PIN_ADR6,
	ID_PIN_ADR7,
	ID_PIN_ADR8};
	
GPIO_TypeDef* STRING_GPIO[6] = {
	GPIOC, 
	GPIOC, 
  GPIOC, 
  GPIOC, 
  GPIOA, 
  GPIOA};

uint16_t STRING_PIN[6] = {
	STRING_PIN_CEA, 
  STRING_PIN_CEB, 
  STRING_PIN_CEC, 
  STRING_PIN_CED, 
  STRING_PIN_CEE, 
  STRING_PIN_CEF};

DMA_Channel_TypeDef* STRING_DMA_CHANNEL[6] = {
	DMA1_Channel1, 
	DMA1_Channel7, 
	DMA1_Channel2, 
	DMA1_Channel3, 
	DMA1_Channel5, 
	DMA1_Channel6};

TIM_TypeDef* STRING_TIM[6] = {
	TIM2, 
  TIM2, 
	TIM3, 
	TIM3, 
	TIM4, 
  TIM3};
	
/* Private functions ---------------------------------------------------------*/
/*void TEST()
{
	unsigned int i,j,k;
	
	i = 2100;	
  while(i>=1000)
	{
		j = 200;

		GPIO_SetBits(STRING_GPIO[0], STRING_PIN[0]);  
			
		while(j)
		{    
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				j = j - 1;
		}  
		
		j = 200;
		GPIO_ResetBits(STRING_GPIO[0], STRING_PIN[0]); 			
		while(j)
		{  
			 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			 j = j - 1;
		}    
		
		i = i - 1; 
	}
	
	DELAY(1500);                                                                                                                                                                                                                       
}	
*/	
void read_ID()
{
	int i;
	
	board_num1 = 0;
	board_num2 = 0;
	for(i = 0;i < 4;i ++)
	{
		if(GPIO_ReadInputDataBit(ADR_GPIO[i], ADR_PIN[i]) == Bit_SET)
		{
			board_num1 |= (1 << i);
		}
	}
	for(i = 0;i < 4;i ++)
	{
		if(GPIO_ReadInputDataBit(ADR_GPIO[i + 4], ADR_PIN[i + 4]) == Bit_SET)
		{
			board_num2 |= (1 << i);
		}
	}
	
	data_buf[1] = board_num1;
	data_buf[2] = board_num2;
}
	
void write_to_data_buf(int num,uint16_t Freq, uint16_t Temp)
{
	data_buf[3 + num * 5 + 1] = (uint8_t)((Freq&0xff00)>>8);
	data_buf[3 + num * 5 + 2] = (uint8_t)(Freq&0x00ff);
	data_buf[3 + num * 5 + 3] = (uint8_t)((Temp&0xff00)>>8);
	data_buf[3 + num * 5 + 4] = (uint8_t)(Temp&0x00ff);
}

float get_temperature(int ch)
{
	int i;
	float result = 0;
	ADC_RegularChannelConfig(ADC1, adc_channel_map[ch], 1, ADC_SampleTime_239Cycles5);			    
  for(i=0;i<8;i++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
		result+=ADC_GetConversionValue(ADC1);
	}
	result=result/8;
	return result;
}

void capture()
{
		int i;
		float temp_log;
	  
	  GPIO_SetBits(GPIOA, STRING_PIN_SWITCH); 
		DELAY(1000);
		for(i = 0;i < 6;i ++)
		{
			if(online_count[i] >= limit_count)
			{
				continue;
			}
			
			MEASUREMENT(i);
			
			state = 1;
			timeout_flag = 1;
			
			TIM_Cmd(STRING_TIM[i], ENABLE); 
			DMA_Cmd(STRING_DMA_CHANNEL[i],ENABLE);			
			/*while((state != 0) && (j != 0))
			{    
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				j = j - 1;
			} */ 
			while((state != 0) && (timeout_flag < 3)) ;

			if(state == 1) //haven't tested
			{
				TIM_Cmd(STRING_TIM[i], DISABLE); 
				DMA_Cmd(STRING_DMA_CHANNEL[i],DISABLE);
			}
			
			state = 0;
			timeout_flag = 0;
		}
		while(sending != 0) ;
		for(i = 0;i < 6; i ++)
		{
			if(online_count[i] >= limit_count)
			{
				write_to_data_buf(i,0,0);	
				continue;
			}
			
			Frequency=240000000.0f/cycleaverage[i] + 0.5f;	//10倍真正频率
			Temperature = get_temperature(i);
			
			if(((Frequency_last[i] - Frequency) >= 1000) || ((Frequency - Frequency_last[i]) >= 1000))
			{
				online_count[i] ++;
				STIMULATE_HIGH[i] = 2200;
				STIMULATE_LOW[i] = 200;
			}
			else
			{
				online_count[i] = 0;
				STIMULATE_HIGH[i] = 240000000.0f / (Frequency - 0.5f - 1000.0f) / 26.0f;
				STIMULATE_LOW[i] = 240000000.0f / (Frequency - 0.5f + 1000.0f) / 26.0f;
				if(STIMULATE_LOW[i] < 0 || STIMULATE_HIGH[i] < 0)
				{
					STIMULATE_LOW[i] = 200;
					STIMULATE_HIGH[i] = 400;
				}
				else if(STIMULATE_HIGH[i] > 3000 || STIMULATE_LOW[i] > 3000)
				{
				  STIMULATE_LOW[i] = 2000;
					STIMULATE_HIGH[i] = 2200;
				}
			}
			Frequency_last[i] = Frequency;
			
			temp_log = log(Temperature * 4.5185f);
			Temperature = (1.0f / (A + B * temp_log + C * temp_log * temp_log * temp_log) - 273.2f) * 100.0f; 
			
			write_to_data_buf(i,(uint16_t)(Frequency),(uint16_t)(Temperature));	
		}
	  GPIO_ResetBits(GPIOA, STRING_PIN_SWITCH);
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
	
  RCC_Configuration();
  NVIC_Configuration();
  GPIO_Configuration();
	SPI_Configuration();
	ADC_Configuration();
  init_timer();
	dma_NVIC();
	init_dma();
	USART_Configuration();

  GPIO_SetBits(GPIOA, ZIGBEE_PIN_RESET); //开zigbee
	GPIO_ResetBits(GPIOD, RS485_DE); 
	
	read_ID();
	
	start_buf[1] = data_buf[1];
	start_buf[2] = data_buf[2];
	reply_buf[1] = data_buf[1];
	reply_buf[2] = data_buf[2];
	
	capture_flag = 1;

  while (1)
	{
		if(capture_flag == 1)
		{
			capture();
			capture_flag = 0;
			if(UART_SendEnum == SEND_DATA)
			{
				if(send_flag == 1)
				{
					uart_zigbee_senddata(data_buf);
					send_flag = 0;
				}					
			}
		}
	  
		
		//TEST();
		
		//capture();
		
		/*GPIO_SetBits(GPIOD, RS485_DE); 
	  uart_485_senddata(data_buf, 36);
		GPIO_ResetBits(GPIOD, RS485_DE);*/
	}
	
}

/*void get_channel(int ch)
{
	switch(ch)
	{
		case 0:
		{
			STRING_GPIO = GPIOC;
			STRING_PIN = GPIO_Pin_8;
			STRING_TIM = TIM2;
			STRING_DMA_CHANNEL = DMA1_Channel1;
			break;
		}
		case 1:
		{
			STRING_GPIO = GPIOC;
			STRING_PIN = GPIO_Pin_9;
			STRING_TIM = TIM2;
			STRING_DMA_CHANNEL = DMA1_Channel7;
			break;
		}
		case 2:
		{
			STRING_GPIO = GPIOA;
			STRING_PIN = GPIO_Pin_8;
			STRING_TIM = TIM3;
			STRING_DMA_CHANNEL = DMA1_Channel2;
			break;
		}
		case 3:
		{
			STRING_GPIO = GPIOA;
			STRING_PIN = GPIO_Pin_9;
			STRING_TIM = TIM3;
			STRING_DMA_CHANNEL = DMA1_Channel3;
			break;
		}
		case 4:
		{
			STRING_GPIO = GPIOA;
			STRING_PIN = GPIO_Pin_10;
			STRING_TIM = TIM4;
			STRING_DMA_CHANNEL = DMA1_Channel5;
			break;
		}
		case 5:
		{
			STRING_GPIO = GPIOA;
			STRING_PIN = GPIO_Pin_11;
			STRING_TIM = TIM3;
			STRING_DMA_CHANNEL = DMA1_Channel6;
			break;
		}
		default:break;
	}
}*/

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void dma_NVIC(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

 void init_dma(void)
 { 
    DMA_InitTypeDef DMA_InitStructure;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	 
	 /*PHZ1*/
	  DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM2->CCR3);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture[0];
	 	DMA_Init(DMA1_Channel1,&DMA_InitStructure);              
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	  /*PHZ2*/
	 	DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM2->CCR4);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture[1];
	  DMA_Init(DMA1_Channel7,&DMA_InitStructure);              
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
		
		/*PHZ3*/
	  DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM3->CCR3);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture[2];
 	  DMA_Init(DMA1_Channel2,&DMA_InitStructure);              
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
		
		/*PHZ4*/
	  DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM3->CCR4);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture[3];
 	  DMA_Init(DMA1_Channel3,&DMA_InitStructure);              
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
		
		/*PHZ5*/
	  DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM4->CCR3);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture[4];
 	  DMA_Init(DMA1_Channel5,&DMA_InitStructure);              
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
		
		/*PHZ6*/
	  DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM3->CCR1);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture[5];
    DMA_Init(DMA1_Channel6,&DMA_InitStructure);              
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA1_Channel1,DISABLE);	 
		DMA_Cmd(DMA1_Channel7,DISABLE);	 
		DMA_Cmd(DMA1_Channel2,DISABLE);	 
		DMA_Cmd(DMA1_Channel3,DISABLE);	 
		DMA_Cmd(DMA1_Channel5,DISABLE);	 
		DMA_Cmd(DMA1_Channel6,DISABLE);	 
 }

 void init_timer(void)
{
    TIM_TimeBaseInitTypeDef timInitStruct;
    TIM_ICInitTypeDef tim_icinit;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
  
	  timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;                
    timInitStruct.TIM_Prescaler = 0;                    
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;     
    timInitStruct.TIM_RepetitionCounter = 0;  
    timInitStruct.TIM_Period = 0xffff; 
	
	  TIM_DeInit(TIM2);
    TIM_InternalClockConfig(TIM2);                                
	  TIM_TimeBaseInit(TIM2, &timInitStruct);
  
	  TIM_DeInit(TIM3);
    TIM_InternalClockConfig(TIM3);                                
	  TIM_TimeBaseInit(TIM3, &timInitStruct);
	
		TIM_DeInit(TIM4);
    TIM_InternalClockConfig(TIM4);                                
	  TIM_TimeBaseInit(TIM4, &timInitStruct);
		
		timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;                
    timInitStruct.TIM_Prescaler = 23999;                    
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;     
    timInitStruct.TIM_RepetitionCounter = 0;  
    timInitStruct.TIM_Period = 999; 
		
		TIM_DeInit(TIM6);
    TIM_InternalClockConfig(TIM6);                                
	  TIM_TimeBaseInit(TIM6, &timInitStruct);
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    
    tim_icinit.TIM_ICFilter = 0x0;
    tim_icinit.TIM_ICPolarity = TIM_ICPolarity_Falling;
    tim_icinit.TIM_ICPrescaler= TIM_ICPSC_DIV1;
    tim_icinit.TIM_ICSelection = TIM_ICSelection_DirectTI;
    
		tim_icinit.TIM_Channel = TIM_Channel_3;
	  TIM_ICInit(TIM2,&tim_icinit); 
		
		tim_icinit.TIM_Channel = TIM_Channel_4;
	  TIM_ICInit(TIM2,&tim_icinit); 
    
		tim_icinit.TIM_Channel = TIM_Channel_3;
	  TIM_ICInit(TIM3,&tim_icinit); 
		
		tim_icinit.TIM_Channel = TIM_Channel_4;
	  TIM_ICInit(TIM3,&tim_icinit); 
		
		tim_icinit.TIM_Channel = TIM_Channel_3;
	  TIM_ICInit(TIM4,&tim_icinit); 
		
		tim_icinit.TIM_Channel = TIM_Channel_1;
	  TIM_ICInit(TIM3,&tim_icinit); 
		
		TIM_ARRPreloadConfig(TIM2, DISABLE);  
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
		
		TIM_ARRPreloadConfig(TIM3, DISABLE);  
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
    
		TIM_ARRPreloadConfig(TIM4, DISABLE);  
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
		
		TIM_DMAConfig(TIM2,TIM_DMABase_CCR3,TIM_DMABurstLength_1Byte);
    TIM_DMACmd(TIM2, TIM_DMA_CC3, ENABLE);
		
		TIM_DMAConfig(TIM2,TIM_DMABase_CCR4,TIM_DMABurstLength_1Byte);
    TIM_DMACmd(TIM2, TIM_DMA_CC4, ENABLE);
		
		TIM_DMAConfig(TIM3,TIM_DMABase_CCR3,TIM_DMABurstLength_1Byte);
    TIM_DMACmd(TIM3, TIM_DMA_CC3, ENABLE);
		
		TIM_DMAConfig(TIM3,TIM_DMABase_CCR4,TIM_DMABurstLength_1Byte);
    TIM_DMACmd(TIM3, TIM_DMA_CC4, ENABLE);
		
		TIM_DMAConfig(TIM4,TIM_DMABase_CCR3,TIM_DMABurstLength_1Byte);
    TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);
		
		TIM_DMAConfig(TIM3,TIM_DMABase_CCR1,TIM_DMABurstLength_1Byte);
    TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);
		
		TIM_Cmd(TIM2, DISABLE); 
		TIM_Cmd(TIM3, DISABLE); 
		TIM_Cmd(TIM4, DISABLE); 
		TIM_Cmd(TIM6, ENABLE);
}
/**
  * @brief  Configure the GPIOD Pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*??SWD ??JTAG*/

  /* DMA */
  GPIO_InitStructure.GPIO_Pin =  STRING_PIN_PHZ1|STRING_PIN_PHZ2|STRING_PIN_PHZ6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin =  STRING_PIN_PHZ3|STRING_PIN_PHZ4|STRING_PIN_PHZ5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* output */
  GPIO_InitStructure.GPIO_Pin = ZIGBEE_PIN_RESET|STRING_PIN_CEE|STRING_PIN_CEF|STRING_PIN_SWITCH;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = STRING_PIN_CEA|STRING_PIN_CEB|STRING_PIN_CEC|STRING_PIN_CED;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RS485_DE;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Configure PB12-15 of spi mode */
	GPIO_InitStructure.GPIO_Pin =  SPIx_PIN_SCK | SPIx_PIN_MOSI;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  SPIx_PIN_NSS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPIx_PIN_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);
	
	
	/* Configure ADR SMR7010 -----------------------------------*/
	GPIO_InitStructure.GPIO_Pin =  ID_PIN_ADR1|ID_PIN_ADR2|ID_PIN_ADR3|ID_PIN_ADR4|ID_PIN_ADR5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  ID_PIN_ADR6|ID_PIN_ADR7|ID_PIN_ADR8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Configure PC0-3 as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = STRING_PIN_TEMPOP1 | STRING_PIN_TEMPOP2 | STRING_PIN_TEMPOP3 | STRING_PIN_TEMPOP4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Configure PA0-1 as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = STRING_PIN_TEMPOP5 | STRING_PIN_TEMPOP6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void SPI_Configuration(void)
{
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_SSOutputCmd(SPIx, ENABLE);
  SPI_Init(SPIx, &SPI_InitStructure);
}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));	
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
}

void USART_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	USART_InitStructure.USART_BaudRate = 19200; //然而实际波特率是9600，不知道发生了啥
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Enable GPIO clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //前面已经开过了

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART3, &USART_InitStructure);
	USART_Init(UART4, &USART_InitStructure);
	USART_Init(USART1, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART3, ENABLE);
	USART_Cmd(UART4, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */


void MEASUREMENT(int group)
{
	unsigned int i,j;
	
	i = STIMULATE_HIGH[group];	
  while(i>=STIMULATE_LOW[group])
	{
		j = i;
		GPIO_SetBits(STRING_GPIO[group], STRING_PIN[group]); 		
		while(j)
		{    
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				j = j - 1;
		}  
		
		j = i;
		GPIO_ResetBits(STRING_GPIO[group], STRING_PIN[group]); 	
		while(j)
		{  
			 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			 j = j - 1;
		}    
		
		i = i - 1; 
	}
	
	DELAY(4000);                                                                                                                                                                                                                       
}

void DELAY(__IO uint32_t nCount)
{
  	unsigned int i,j;
	
  	for(i=0;i<50;i++)
  	{
	  	for(j=0;j<nCount;j++)
   		{
	   		__nop();
    	}
   } 	
}

void uart_zigbee_senddata(uint8_t *str)
{
	int i;
	for(i = 0;i < 36;i ++)
	{
		USART_SendData(USART3, *(str + i));

		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		{}
	}
}

void uart_485_senddata(uint8_t *str, uint8_t num)
{
	int i;
	for(i = 0;i < num;i ++)
	{
		USART_SendData(UART4, *(str + i));

		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
		{}
	}
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
