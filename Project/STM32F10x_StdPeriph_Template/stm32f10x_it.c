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
extern volatile uint8_t timeout_flag;
extern volatile uint8_t sending;
extern volatile uint8_t send_flag;
extern volatile uint8_t address_flag;
extern UART_SendTypeDef last_read;

extern uint8_t data_buf[36];
extern uint8_t reply_buf[35];
extern uint8_t start_buf[35];

extern char mode;

volatile extern UART_SendTypeDef UART_SendEnum;
volatile extern uint8_t zigbee_connect_flag;
extern unsigned char board_num1;
extern unsigned char board_num2;


uint32_t cycle[30] = {0};
__IO uint32_t allcycle = 0;
int interval = 1;
int count = 0;
uint8_t address_interval = 30;
uint8_t address_count = 2;
uint8_t reset_flag = 0;
uint8_t reset_count = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void write_flash(uint16_t dat)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(FLASH_START_ADDR);
	FLASH_ProgramHalfWord(FLASH_START_ADDR,dat);
}

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
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //?? TIM3 ????????  
	{  
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //?? TIM3 ??????  
		
		count ++;
		if(count >= interval)
		{
			count = 0;
			send_flag = 1;
		}
		
		address_count ++;
		if(address_count > address_interval)
		{
			address_count = 0;
			address_flag = 1;
		}
		
		if(reset_flag == 1)
		{
			reset_count ++;
			if(reset_count >= 5)
			{
				reset_flag = 0;
				reset_count = 0;
				GPIO_SetBits(GPIOA, ZIGBEE_PIN_RESET);
			}
		}
		
		if(timeout_flag != 0)
		{
			timeout_flag ++;
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

uint8_t recv;
uint8_t rx_buf[10] = {0};
uint8_t rx_count = 0;
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
		  USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
		  recv = USART_ReceiveData(USART1);
		
		  if(rx_count == 0)
	    {
				if(recv == 'S')
				{
					rx_buf[0] = recv;
					rx_count ++;
				}
				return ;
	    }
	    else if(rx_count == 1)
			{
				rx_buf[rx_count] = recv;
	      rx_count ++;
				return ;
			}
			else if(rx_count == 2)
			{
				rx_buf[rx_count] = recv;
	      rx_count ++;
				return ;
			}
			else if(rx_count > 2)
			{
				rx_buf[rx_count] = recv;
				rx_count ++;
				if(recv == 'E')
				{
					if(rx_count == 4)
					{
						data_buf[33] = rx_buf[1];
						data_buf[34] = rx_buf[2];
					}
					rx_count = 0;
				}
			}
	}
}

uint8_t data;
uint8_t rxbuf[40] = {0};
volatile uint8_t rxcount = 0;
void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
		  USART_ClearITPendingBit(UART4,USART_IT_RXNE); 
		  data = USART_ReceiveData(UART4);
		
		  if(rxcount == 0)
	    {
				if(data == 'H')
				{
					rxbuf[rxcount] = data;
					rxcount ++;
				}
				return ;
	    }
			else if(rxcount > 0)
			{
				rxbuf[rxcount] = data;
				rxcount ++;
				if(data == 'E')
				{
					if(rxcount == 35)
					{
						if(rxbuf[11] == board_num1 && rxbuf[12] == board_num2)
						{
							//TODO:在此添加命令解析代码
							 if(rxbuf[18] != 0xff)
							 {
								 //TODO:设置采样间隔 
								 interval = rxbuf[18] * 0.5;
								 reply_buf[32] = 0;
								 reply_buf[33] = 0;
								 reply_buf[34] = 0x55;
							 }
							 else if(rxbuf[16] == 0xff)
							 {
								 mode = 'R';
								 reply_buf[32] = 0xff;
								 reply_buf[33] = 0;
								 reply_buf[34] = 0xff;
								 
							 }
							 else if(rxbuf[26] == 0)
							 {
							   mode = 'Z';
								 reply_buf[32] = 0x55;
								 reply_buf[33] = 0;
								 reply_buf[34] = 0;
							 }						
							 GPIO_SetBits(GPIOD, RS485_DE); 
							 uart_485_senddata(reply_buf, 36);
							 GPIO_ResetBits(GPIOD, RS485_DE);
				       if(mode == 'R')
							 {
								 UART_SendEnum = SEND_DATA;
								 write_flash(53);
							 }
							 else if(mode == 'Z')
							 {
					       UART_SendEnum = SEND_SLEEP;
								 write_flash(45);
							 }
						 }
					}
					rxcount = 0;
				}
			}
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

			if((zigbee_connect_flag == 0) && (dat == 'K'))
	    {
				zigbee_connect_flag = 1;
				uart_zigbee_senddata(start_buf);
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
							 {
								 UART_SendEnum = SEND_DATA;
								 write_flash(53);
							 }
							 else if(mode == 'Z')
							 {
					       UART_SendEnum = SEND_SLEEP;
								 write_flash(45);
							 }
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
							zigbee_connect_flag = 0;
							GPIO_ResetBits(GPIOA, ZIGBEE_PIN_RESET); 
							reset_flag = 1;
							reset_count = 0;
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
