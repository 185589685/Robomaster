/*
*****************************************************************************
*@file    Basis_DR16.c
*@author  Dr.Wu
*@version V1.0
*@date
*@brief
*
*****************************************************************************
*@attention
*
*****************************************************************************
*/
#include "Basis_DR16.h"
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

uint8_t DR16Buffer[DR16BufferNumber];
uint8_t DR16Buffer[22];

DR16_t DR16 = DR16_GroundInit;


uint8_t testdatatosend[50];//匿名上位机发送数据


void usart1_dr16_init(void)
{
	/*清空标志位然后使能USART的中断*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	/*开启DMA传输（但是不开启DMA中断）*/
	USART_Receive_DMA_NO_IT(&huart1, DR16Buffer, DR16BufferNumber);
}
/**
  * @Data    2019-02-19 15:46
  * @brief   USART_DMA接收开启和重定向
  * @param   void
  * @retval  void
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

	/*检测当前huart状态*/
	if (huart->RxState == HAL_UART_STATE_READY)
	{
		/*输入的地址或者数据有问题的话*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		/*huart里面对应的Rx变量重定向*/
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode = HAL_UART_ERROR_NONE;

		/*开启huart1上的RX_DMA*/
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/*只开启对应DMA上面的Rx功能（如果是开启Tx的话就是USART_CR3_DMAT）*/
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	}
	else
	{
		return HAL_BUSY;
	}

	return HAL_OK;
}


