
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "cmsis_os.h"
//#include "Hub_Data.h"
#include "HC0506_USART.h"


static void HC0506_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static void HC0506_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static int 	HC0506_Wait_Receive_Data(UART_HandleTypeDef *huart, uint16_t size, int timeout_before_response, int timeout_between_packet);
static int 	HC0506_Recieve_Timeout(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size, int timeout_before_response, int timeout_between_packet);
static void HC0506_Rx_DMA_Reset(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size);
static int 	HC0506_Get_Rx_Count(UART_HandleTypeDef *huart, uint16_t Size);

/* Callbacks structure for GSM core */
static const struct link_hc0506_callbacks hc0506_callbacks = {
	HC0506_Transmit,
	HC0506_Receive,
	HC0506_Wait_Receive_Data,
	HC0506_Recieve_Timeout,
	HC0506_Rx_DMA_Reset,
	HC0506_Get_Rx_Count
};

/*###############################################################*/
/*###############################################################* HC0506_Transmit -->*/
/*###############################################################*/
static void HC0506_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status;

	while(huart->gState != HAL_UART_STATE_READY)
	{
		osDelay(1);
	}	
	
	status=HAL_UART_Transmit_DMA(huart, pData, Size);
	#ifdef HUB_DEBUG_BLUETOOTH
	if (status==HAL_ERROR)
	{
		HUB_DEBUG_BLUETOOTH(("\r\nUSART Transmit HAL_ERROR\r\n"));
	}
	else if (status==HAL_BUSY)
	{
		HUB_DEBUG_BLUETOOTH(("\r\nUSART Transmit HAL_BUSY\r\n"));
	}
	else if (status==HAL_TIMEOUT)
	{
		HUB_DEBUG_BLUETOOTH(("\r\nUSART Transmit HAL_TIMEOUT\r\n"));
	}
	#endif
}

/*###############################################################*/
/*###############################################################* HC0506_Receive -->*/
/*###############################################################*/
static void HC0506_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status;
	
	while(huart->gState != HAL_UART_STATE_READY)
	{
		osDelay(1);
	}	
	
	status=HAL_UART_Receive_DMA(huart, pData, Size);
	#ifdef HUB_DEBUG_BLUETOOTH
	if (status==HAL_ERROR)
	{
		HUB_DEBUG_BLUETOOTH(("\r\nUSART Receive HAL_ERROR\r\n"));
	}
	else if (status==HAL_BUSY)
	{
		HUB_DEBUG_BLUETOOTH(("\r\nUSART Receive HAL_BUSY\r\n"));
	}
	#endif
}

/*###############################################################*/
/*###############################################################* HC0506_Wait_Receive_Data -->*/
/*###############################################################*/
static int HC0506_Wait_Receive_Data(UART_HandleTypeDef *huart, uint16_t size, int timeout_before_response, int timeout_between_packet)
{
		int len=0;
		int len_last=0;
		int tmt_pct=timeout_between_packet;
	
	while(len<2 && timeout_before_response>0) //len<2 - мало ли помеху какую споймали
		{			
			osDelay(1);
			len=size - huart->hdmarx->Instance->NDTR;
			timeout_before_response--;	
		}	
		if ((len==0)||(timeout_before_response==0))goto exit;

		while(tmt_pct>0)
		{			
			osDelay(1);
			len=size - huart->hdmarx->Instance->NDTR; 
			if(len==len_last)tmt_pct--;
			if(len>len_last)tmt_pct = timeout_between_packet;	
			if(len==size)goto exit;
			len_last=len;						
		}
exit:		
	huart->RxState = HAL_UART_STATE_READY;
	return len;
}

/*###############################################################*/
/*###############################################################* HC0506_Recieve_Timeout -->*/
/*###############################################################*/
static int HC0506_Recieve_Timeout(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size, int timeout_before_response, int timeout_between_packet)
{
	HC0506_Receive(huart, pData, size);
	return HC0506_Wait_Receive_Data(huart, size, timeout_before_response, timeout_between_packet);
}

/*###############################################################*/
/*###############################################################* HC0506_Rx_DMA_Reset -->*/
/*###############################################################*/
static void HC0506_Rx_DMA_Reset(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size)
{
	__HAL_DMA_DISABLE(huart->hdmarx);/* отключаем дма */
    /* Clear all flags */
    __HAL_DMA_CLEAR_FLAG (huart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
    __HAL_DMA_CLEAR_FLAG (huart->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(huart->hdmarx));
    __HAL_DMA_CLEAR_FLAG (huart->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(huart->hdmarx));
    __HAL_DMA_CLEAR_FLAG (huart->hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(huart->hdmarx));
    __HAL_DMA_CLEAR_FLAG (huart->hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(huart->hdmarx));
	huart->hdmarx->Instance->NDTR = size; /* размер дпнных */
  huart->hdmarx->Instance->M0AR = (uint32_t)buff;	/* адрес массива */
	memset(buff, 0, size);
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_DMA_ENABLE(huart->hdmarx);/* включаем дма */	
}

/*###############################################################*/
/*###############################################################* HC0506_Get_Rx_Count -->*/
/*###############################################################*/
static int HC0506_Get_Rx_Count(UART_HandleTypeDef *huart, uint16_t Size)
{
	return Size - huart->hdmarx->Instance->NDTR;
}
/*###############################################################*/
/*###############################################################* HC0506_USART_Change_BaudRate -->*/
/*###############################################################*/
void HC0506_USART_Change_BaudRate(UART_HandleTypeDef *huart, uint32_t BaudRate)
{
  if (HAL_UART_DeInit(huart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  huart->Init.BaudRate = BaudRate;

  if (HAL_UART_Init(huart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
/*###############################################################*/
/*###############################################################* link_hc0506_callbacks -->*/
/*###############################################################*/
const struct link_hc0506_callbacks* HC0506_Get_Link_Callbacks(void)
{
	return &hc0506_callbacks;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){}
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	while(1){
//		printf("FULL PPPOS BUFFER");
//		osDelay(1);
//	}
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
		//Error_Handler();
}
void HAL_UART_AbortCpltCallback (UART_HandleTypeDef *huart){Error_Handler();}
void HAL_UART_AbortTransmitCpltCallback (UART_HandleTypeDef *huart){Error_Handler();}
void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart){Error_Handler();}

