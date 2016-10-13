/**
  ******************************************************************************
  * @file    ADC3_DMA/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
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
#include "stm32f4xx_it.h"
#include <App.h>
/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup ADC_ADC3_DMA
  * @{
  */  

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
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
	printf("HF:%s:%d\r\n", App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskName.c_str(), App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->duration[1] - App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->duration[0]);
	App::mTicks->PrintTime();
	while(true);
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
  {
  }
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
  {
  }
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
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void){
	App::mApp->mTicks->TicksIncrement();
	if(App::mApp->mTicks->getTicks() >= Ticks::maxTicks){
		App::mApp->mTicks->setTicks(0);
	}

	if(App::mApp->mTicks->getTicks() % 1000 == 0){
		App::mTicks->Sec++;
	}

	if(App::mApp->mTask != 0 && App::mApp->mTask->IsPrintTaskNum){
		if(App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->hangCount++ > App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskPeriod){
			static int delayCount = 0;
			if(delayCount++ > 10){
				delayCount = 0;
				printf("%s:%d\r\n", App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskName.c_str(), App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->hangCount);
			}
		}
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
	{
		DMA_ClearITPendingBit (DMA1_Stream3, DMA_IT_TCIF3);
		DMA_Cmd(DMA1_Stream3, DISABLE);
		App::mApp->mTicks->setTimeout(3);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){
			if(App::mApp->mTicks->Timeout()){
				return;
			}
		}
		for(int i = 0; i < 5; i++){
			App::mApp->mSpi2->Buffer[App::mApp->mSpi2->BufferCount++] = App::mApp->mSpi2->rxBuffer[i];
			App::mApp->mSpi2->AvailableLength++;
			if(App::mApp->mSpi2->BufferCount >= 2047){
				App::mApp->mSpi2->BufferCount = 0;
			}
		}
		DMA_Cmd(DMA1_Stream3, ENABLE);
	}

//	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
//	{
//		DMA_ClearITPendingBit (DMA1_Stream3, DMA_IT_TCIF3);
//		DMA_Cmd(DMA1_Stream3, DISABLE);
//		App::mApp->mTicks->setTimeout(3);
//		while (USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET){
//			if(App::mApp->mTicks->Timeout()){
//				break;
//			}
//		}
//		USART_ClearFlag(USART3,USART_FLAG_TC);
//		App::mApp->mUART3->isDmaBusy = false;//setIsDmaBusy(false);
//	}
}





/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

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


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
