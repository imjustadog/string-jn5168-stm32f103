/**
  ******************************************************************************
  * @file    TIM/InputCapture/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "project.h"
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
extern void DELAY(__IO uint32_t nCount);
extern void uart_zigbee_senddata(uint8_t *str);
extern void uart_485_senddata(uint8_t *str, uint8_t num);

extern float cycleaverage[6];
extern uint16_t Capture[6][30];
extern uint32_t size;

extern volatile uint8_t state;
extern volatile uint8_t sending;
extern volatile uint8_t capture_flag;
extern volatile uint8_t send_flag;

extern uint8_t data_buf[36];
extern uint8_t reply_buf[35];

extern char mode;

extern UART_SendTypeDef UART_SendEnum;
extern unsigned int board_num;

uint32_t cycle[30] = {0};
__IO uint32_t allcycle = 0;
int interval = 8;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
int count = 0;
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //?? TIM3 ????????  
	{  
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update ); //?? TIM3 ??????  
		count ++;
		if(count >= interval)
		{
			count = 0;
			capture_flag = 1;
			send_flag = 1;
		}
	}
}	
	
void DMA1_Channel1_IRQHandler(void)
{ 
	unsigned char i=0,j=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1) == SET) 
  {
		DMA_ClearFlag(DMA1_FLAG_TC1);
		j = 0;
		
		allcycle=0;
		for(i=1;i<size-1;i++)
		{
			if (Capture[j][i+1] > Capture[j][i])
			{
				cycle[i] = (Capture[j][i+1] - Capture[j][i]); 
			}
			else
			{
				cycle[i]= ((0xFFFF - Capture[j][i]) + Capture[j][i+1]); 
			}
			/* Frequency computation */ 
			allcycle += cycle[i];
		}

		cycleaverage[j] = allcycle  /(float)(size-2);
    state = 0;
		TIM_Cmd(TIM2, DISABLE); 
    DMA_Cmd(DMA1_Channel1,DISABLE);	
  }
}

void DMA1_Channel7_IRQHandler(void)
{ 
	unsigned char i=0,j=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7) == SET) 
  {
		DMA_ClearFlag(DMA1_FLAG_TC7);
		j = 1;
		
		allcycle=0;
		for(i=1;i<size-1;i++)
		{
			if (Capture[j][i+1] > Capture[j][i])
			{
				cycle[i] = (Capture[j][i+1] - Capture[j][i]); 
			}
			else
			{
				cycle[i]= ((0xFFFF - Capture[j][i]) + Capture[j][i+1]); 
			}
			/* Frequency computation */ 
			allcycle += cycle[i];
		}

		cycleaverage[j] = allcycle  /(float)(size-2);
		state = 0;
		TIM_Cmd(TIM2, DISABLE); 
    DMA_Cmd(DMA1_Channel7,DISABLE);	
  }
}

void DMA1_Channel2_IRQHandler(void)
{ 
	unsigned char i=0,j=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC2) == SET) 
  {
		DMA_ClearFlag(DMA1_FLAG_TC2);
		j = 2;
		
		allcycle=0;
		for(i=1;i<size-1;i++)
		{
			if (Capture[j][i+1] > Capture[j][i])
			{
				cycle[i] = (Capture[j][i+1] - Capture[j][i]); 
			}
			else
			{
				cycle[i]= ((0xFFFF - Capture[j][i]) + Capture[j][i+1]); 
			}
			/* Frequency computation */ 
			allcycle += cycle[i];
		}

		cycleaverage[j] = allcycle  /(float)(size-2);
		state = 0;
		TIM_Cmd(TIM3, DISABLE); 
    DMA_Cmd(DMA1_Channel2,DISABLE);	
  }
}

void DMA1_Channel3_IRQHandler(void)
{ 
	unsigned char i=0,j=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC3) == SET) 
  {
		DMA_ClearFlag(DMA1_FLAG_TC3);
		j = 3;
		
		allcycle=0;
		for(i=1;i<size-1;i++)
		{
			if (Capture[j][i+1] > Capture[j][i])
			{
				cycle[i] = (Capture[j][i+1] - Capture[j][i]); 
			}
			else
			{
				cycle[i]= ((0xFFFF - Capture[j][i]) + Capture[j][i+1]); 
			}
			/* Frequency computation */ 
			allcycle += cycle[i];
		}

		cycleaverage[j] = allcycle  /(float)(size-2);
		state = 0;
		TIM_Cmd(TIM3, DISABLE); 
    DMA_Cmd(DMA1_Channel3,DISABLE);	
  }
}

void DMA1_Channel5_IRQHandler(void)
{ 
	unsigned char i=0,j=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET) 
  {
		DMA_ClearFlag(DMA1_FLAG_TC5);
		j = 4;
		
		allcycle=0;
		for(i=1;i<size-1;i++)
		{
			if (Capture[j][i+1] > Capture[j][i])
			{
				cycle[i] = (Capture[j][i+1] - Capture[j][i]); 
			}
			else
			{
				cycle[i]= ((0xFFFF - Capture[j][i]) + Capture[j][i+1]); 
			}
			/* Frequency computation */ 
			allcycle += cycle[i];
		}

		cycleaverage[j] = allcycle  /(float)(size-2);
		state = 0;
		TIM_Cmd(TIM4, DISABLE); 
    DMA_Cmd(DMA1_Channel5,DISABLE);	
  }
}

void DMA1_Channel6_IRQHandler(void)
{ 
	unsigned char i=0,j=0;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC6) == SET) 
  {
		DMA_ClearFlag(DMA1_FLAG_TC6);
		j = 5;
		
		allcycle=0;
		for(i=1;i<size-1;i++)
		{
			if (Capture[j][i+1] > Capture[j][i])
			{
				cycle[i] = (Capture[j][i+1] - Capture[j][i]); 
			}
			else
			{
				cycle[i]= ((0xFFFF - Capture[j][i]) + Capture[j][i+1]); 
			}
			/* Frequency computation */ 
			allcycle += cycle[i];
		}

		cycleaverage[j] = allcycle  /(float)(size-2);
		state = 0;
		TIM_Cmd(TIM3, DISABLE); 
    DMA_Cmd(DMA1_Channel6,DISABLE);		
  }
}

uint8_t data;
uint8_t rxbuf[40] = {0};
uint8_t rxcount = 0;
void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
		  USART_ClearITPendingBit(UART4,USART_IT_RXNE); 
		  data = USART_ReceiveData(UART4);
		
		  if((rxcount == 0) && (data == 'S'))
	    {
				rxbuf[0] = data;
				rxcount = 1;
	    }
	    else if(data == 'E')
			{
				rxbuf[rxcount] = data;
	      rxcount ++;
				if((rxcount == 6) && (rxbuf[1] == (board_num >> 8)) && (rxbuf[2] == (board_num & 0x00ff)))
				{
					data_buf[33] = rxbuf[3];
					data_buf[34] = rxbuf[4];
				}
				else if(rxcount == 3)
				{
					 GPIO_SetBits(GPIOD, RS485_DE); 
					 sending = 1;
					 uart_485_senddata(data_buf, 36);
					 sending = 0;
					 GPIO_ResetBits(GPIOD, RS485_DE);
				}
				rxcount = 0;
			}
			else if(data == 'E')
			{
			}
			else if(rxcount > 0)
			{
				rxbuf[rxcount] = data;
				rxcount ++;
				if(rxcount > 14) rxcount = 14;
			}
			else ;
	}
}

uint8_t dat;
uint8_t start_judge = 0;
uint8_t receive_buf[15] = {0};
uint8_t receive_count = 0;
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
		  USART_ClearITPendingBit(USART3,USART_IT_RXNE); 
    /* Read one byte from the receive data register */
		  dat = USART_ReceiveData(USART3);

			if((UART_SendEnum == SEND_NONE) && (dat == 'K'))
	    {
				UART_SendEnum = SEND_START;
				return;
	    }
		
		  if((start_judge == 0) && (dat == 'S'))
	    {
				start_judge = 1;
				receive_buf[0] = dat;
				receive_count ++;
	    }
	    else if(start_judge == 1)
			{
				receive_buf[receive_count] = dat;
	      receive_count ++;
  	    if(receive_count == 15)
  	    {
	      	  receive_count = 0;
	          start_judge = 0; 
					  if(dat == 'E')
						{
							//TODO:在此添加命令解析代码
							 if(receive_buf[4] != 0xff)
							 {
								 //TODO:设置采样间隔 
								 interval = receive_buf[4] * 0.5;
								 reply_buf[32] = 0;
								 reply_buf[33] = 0;
								 reply_buf[34] = 0x55;
							 }
							 else if(receive_buf[2] == 0xff)
							 {
								 mode = 'R';
								 reply_buf[32] = 0xff;
								 reply_buf[33] = 0;
								 reply_buf[34] = 0xff;
								 
							 }
							 else if(receive_buf[12] == 0)
							 {
							   mode = 'Z';
								 reply_buf[32] = 0x55;
								 reply_buf[33] = 0;
								 reply_buf[34] = 0;
							 }
							 uart_zigbee_senddata(reply_buf);
				       if(mode == 'R')
								 UART_SendEnum = SEND_DATA;
							 else if(mode == 'Z')
					       UART_SendEnum = SEND_NONE;
						}
				}
			}
			else
			{
				switch(start_judge)
				{
					case 0 :
					{
						if(dat == 'X')
						{
							start_judge = 2;
						}
						break;
					}
					case 2 :
					{
						if(dat == 'T')
						{
							start_judge = 3;
						}
						else
						{
							start_judge = 0;
						}
						break;
					}
					case 3 :
					{
						if(dat == 'Q')
						{
							start_judge = 4;
						}
						else
						{
							start_judge = 0;
						}
						break;
					}
					case 4 :
					{
						if(dat == 'D')
						{
							//TODO:在此添加重启代码
							UART_SendEnum = SEND_NONE;
							GPIO_ResetBits(GPIOA, ZIGBEE_PIN_RESET); 
							DELAY(50000);
							GPIO_SetBits(GPIOA, ZIGBEE_PIN_RESET);
						}
						start_judge = 0;
						break;
					}
					default:break;
				}
	    }
  }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
