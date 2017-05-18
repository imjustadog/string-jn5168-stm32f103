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

GPIO_TypeDef* STRING_GPIO;
TIM_TypeDef* STRING_TIM;
DMA_Channel_TypeDef* STRING_DMA_CHANNEL;
uint16_t STRING_PIN;


/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void SPI_Configuration(void);
void ADC_Configuration(void);
void MEASUREMENT(void);
void DELAY(__IO uint32_t nCount);
void init_timer(void);
void init_dma(void);
void dma_NVIC(void);
void get_channel(int i);
uint16_t Capture[6][30]={0};
volatile uint8_t state=0;
char mode='R';
uint32_t size=25;
float Frequency = 0;
float cycleaverage[6] = {0};
int send_flag = 0;
uint16_t bat_volt = 0;
unsigned int board_num =0xAAAA;

/************zigbee通讯部分******************************/
void uart_senddata(uint8_t *str);
UART_SendTypeDef UART_SendEnum  = SEND_NONE;

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
	
	
uint8_t receive_buf[15] = {0};
int receive_count = 0;
/* Private functions ---------------------------------------------------------*/
void write_to_data_buf(int num,uint16_t Freq)
{
	data_buf[3 + num * 5 + 1] = (uint8_t)((Freq&0xff00)>>8);
	data_buf[3 + num * 5 + 2] = (uint8_t)(Freq&0x00ff);
}
	
void get_battery_voltage()
{
	ADC_Cmd(ADC1, ENABLE);
	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	bat_volt = ADC_GetConversionValue(ADC1);
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	ADC_TempSensorVrefintCmd(DISABLE);
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);
}

void capture()
{
		int i;
	  GPIO_SetBits(GPIOA, STRING_PIN_SWITCH); 
		DELAY(1000);
		for(i = 0;i < 6;i ++)
		{
			get_channel(i);
			MEASUREMENT();
			state = 1;
			TIM_Cmd(STRING_TIM, ENABLE); 
			DMA_Cmd(STRING_DMA_CHANNEL,ENABLE);				
			while(state == 1) ; 
			Frequency=240000000/cycleaverage[i] + 0.5;	//10倍真正频率	
			write_to_data_buf(i,(uint16_t)(Frequency));	
		}
		get_battery_voltage();
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
	
	GPIO_ResetBits(GPIOB, BAT_PIN_1);
	GPIO_ResetBits(GPIOC, BAT_PIN_2);
	GPIO_ResetBits(GPIOA, BAT_PIN_3);
	
	data_buf[1] = (board_num >>8)& 0x00FF;
	data_buf[2] = board_num & 0x00ff;
	start_buf[1] = data_buf[1];
	start_buf[2] = data_buf[2];
	reply_buf[1] = data_buf[1];
	reply_buf[2] = data_buf[2];

  while (1)
	{
		switch(UART_SendEnum)
		{
			case SEND_DATA:
			{
				capture();
				if(UART_SendEnum == SEND_DATA)
					uart_senddata(data_buf);
				break;
			}
			case SEND_NONE:
			{
				break;
			}
			case SEND_START:
			{
				uart_senddata(start_buf);
				uart_senddata(start_buf);
				UART_SendEnum = SEND_DATA;
				break;
			}
			default:break;
		}
	//printf("S1234567890123456789012345678901234");
	}
	
}

void get_channel(int ch)
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
}

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

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void dma_NVIC(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
  GPIO_InitStructure.GPIO_Pin = \
				ZIGBEE_PIN_RESET|STRING_PIN_CEC|STRING_PIN_CED|STRING_PIN_CEE|STRING_PIN_CEF|STRING_PIN_SWITCH|BAT_PIN_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BAT_PIN_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = STRING_PIN_CEA|STRING_PIN_CEB|BAT_PIN_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
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
	
	/* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
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

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);
	
	  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
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

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART3, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART3, ENABLE);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */


void MEASUREMENT(void)
{unsigned int i,j,k;
	
	RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
	i = 2100;	
  while(i>=1000)
	{
		for(k=0;k<1;k++)
		{
          		j = i;
          		GPIO_SetBits(STRING_GPIO, STRING_PIN);  	
          		while(j)
             	{    
              		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								 /* __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();*/
				           j=j-1;
              	}  
          		j = i;
          		GPIO_ResetBits(STRING_GPIO, STRING_PIN);         		
          		while(j)
              	{  
               	 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								 /*__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
								  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();*/
                  	j=j-1;
              	}          
           } 
	i = i-1;
  				 
	}
	
	DELAY(1500);                                                                                                                                                                                                                       
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

void uart_senddata(uint8_t *str)
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
