#include "ANO.h" 
uint8_t DataToSend[1000];


 
 
void ANODT_SendF1(int16_t _a,int16_t _b,int32_t _c)
{
  uint8_t _cnt = 0;
	
  DataToSend[_cnt++] = 0xAA; 
  DataToSend[_cnt++] = 0xFF;
  DataToSend[_cnt++] = 0xF1;
  DataToSend[_cnt++] = 8;
	
  DataToSend[_cnt++] = BYTE0(_a);
  DataToSend[_cnt++] = BYTE1(_a);
  
  DataToSend[_cnt++] = BYTE0(_b);
  DataToSend[_cnt++] = BYTE1(_b);

  DataToSend[_cnt++] = BYTE0(_c);
  DataToSend[_cnt++] = BYTE1(_c);
  DataToSend[_cnt++] = BYTE2(_c);
  DataToSend[_cnt++] = BYTE3(_c);  

  uint8_t sc = 0;
  uint8_t ac = 0;	
	
  for(uint8_t i = 0;i < DataToSend[3]+4;i++){
      sc+=DataToSend[i];
	  ac+=sc;
  }
  DataToSend[_cnt++] = sc;
  DataToSend[_cnt++] = ac;
  
//  for( i = 0;i < _cnt;i++){
//  USART_sendChar(USART6,DataToSend[i]);
//  }
//  for( i = 0;i < _cnt;i++){
// USART6->DR = ((uint8_t)DataToSend[i] & (uint16_t)0x01FF);
//  }
//  USART6->DR = ((uint8_t)DataToSend[_cnt++] & (uint16_t)0x01FF);
  HAL_UART_Transmit_DMA(&huart6,DataToSend,_cnt);
}












































