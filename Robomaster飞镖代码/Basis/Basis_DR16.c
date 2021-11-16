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


uint8_t testdatatosend[50];//������λ����������


void usart1_dr16_init(void)
{
	/*��ձ�־λȻ��ʹ��USART���ж�*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	/*����DMA���䣨���ǲ�����DMA�жϣ�*/
	USART_Receive_DMA_NO_IT(&huart1, DR16Buffer, DR16BufferNumber);
}
/**
  * @Data    2019-02-19 15:46
  * @brief   USART_DMA���տ������ض���
  * @param   void
  * @retval  void
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

	/*��⵱ǰhuart״̬*/
	if (huart->RxState == HAL_UART_STATE_READY)
	{
		/*����ĵ�ַ��������������Ļ�*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		/*huart�����Ӧ��Rx�����ض���*/
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode = HAL_UART_ERROR_NONE;

		/*����huart1�ϵ�RX_DMA*/
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/*ֻ������ӦDMA�����Rx���ܣ�����ǿ���Tx�Ļ�����USART_CR3_DMAT��*/
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	}
	else
	{
		return HAL_BUSY;
	}

	return HAL_OK;
}


