
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

//#include "Hub_Data.h"
//#include "Bluetooth.h"
#include "main.h"
#include "Bluetooth_Interface.h"
#include "HC0506_USART.h"



uint8_t BLE_Rx_Buff[BLE_BUFFER_SIZE];
uint8_t BLE_Tx_Buff[BLE_BUFFER_SIZE];
uint8_t PC_Rx_Buff[BLE_BUFFER_SIZE];
uint8_t PC_Tx_Buff[BLE_BUFFER_SIZE];

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

int To_BLE_Tx_Cnt = 0;
int To_PC_Tx_Cnt = 0;
/*###############################################################*/
/*###############################################################* Get_Rx_Count -->*/
/*###############################################################*/
static int Get_Rx_Count(UART_HandleTypeDef *huart, uint16_t Size)
{
	return Size - huart->hdmarx->Instance->NDTR;
}

/*###############################################################*/
/*###############################################################* Rx_DMA_Reset -->*/
/*###############################################################*/
void Rx_DMA_Reset(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size)
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
	//memset(buff, 0, size);
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_DMA_ENABLE(huart->hdmarx);/* включаем дма */	
}
/*###############################################################*/
/*###############################################################* Get_Rx_Count -->*/
/*###############################################################*/
void *BLE_Receive(void)
{
	static int point_start = 0;
	uint8_t *p = NULL;
	int rx_cnt = Get_Rx_Count(&huart3,BLE_BUFFER_SIZE);
	if((rx_cnt > 0)&&(rx_cnt!=point_start)){
		int len = rx_cnt-point_start;
		p = malloc(len);
		if(p == NULL){printf("ERROR: PC Receive Malloc!!!\r\n");}
		To_PC_Tx_Cnt = len;
		memcpy( (uint8_t*)p, (uint8_t*)&BLE_Rx_Buff[point_start], len);

		for(int i=0; i<len; i++) printf("%c",p[i]);
		
		if (BLE_Rx_Buff[rx_cnt-1]=='\n')
		{
			//memset((uint8_t*)&BLE_Rx_Buff[0],0,rx_cnt);
			Rx_DMA_Reset(&huart3, &BLE_Rx_Buff[0], BLE_BUFFER_SIZE);
			point_start = 0;
			//point_start = rx_cnt;
		}else{
			point_start = rx_cnt;
		}		
	}
	
return p;	
}

/*###############################################################*/
/*###############################################################* Get_Rx_Count -->*/
/*###############################################################*/
void *PC_Receive(void)
{
	static int point_start = 0;
	uint8_t *p = NULL;
	int rx_cnt = Get_Rx_Count(&huart6,BLE_BUFFER_SIZE);
	if((rx_cnt > 0)&&(rx_cnt!=point_start)){
		int len = rx_cnt-point_start;
		p = malloc(len);
		if(p == NULL){printf("ERROR: PC Receive Malloc!!!\r\n");}
		To_BLE_Tx_Cnt = len;
		memcpy( (uint8_t*)p, (uint8_t*)&PC_Rx_Buff[point_start], len);
	
		for(int i=0; i<len; i++) printf("%c",p[i]);
		
		if (PC_Rx_Buff[rx_cnt-1]=='\n')
		{
			//memset((uint8_t*)&PC_Rx_Buff[0],0,rx_cnt);
			Rx_DMA_Reset(&huart6, &PC_Rx_Buff[0], BLE_BUFFER_SIZE);
			point_start = 0;
			//point_start = rx_cnt;
		}else{
			point_start = rx_cnt;
		}		
	}
	
return p;	
}

/*###############################################################*/
/*###############################################################* PC_Transmit -->*/
/*###############################################################*/
void BLE_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status;

	while(huart->gState != HAL_UART_STATE_READY)
	{
		osDelay(1);
	}	
	
	status=HAL_UART_Transmit_DMA(huart, pData, Size);
	if (status==HAL_ERROR)
	{
		printf("\r\nPC USART Transmit HAL_ERROR\r\n");
	}
	else if (status==HAL_BUSY)
	{
		printf("\r\nPC USART Transmit HAL_BUSY\r\n");
	}
	else if (status==HAL_TIMEOUT)
	{
		printf("\r\nPC USART Transmit HAL_TIMEOUT\r\n");
	}
}

/*###############################################################*/
/*###############################################################* PC_Transmit -->*/
/*###############################################################*/
void PC_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status;

	while(huart->gState != HAL_UART_STATE_READY)
	{
		osDelay(1);
	}	
	
	status=HAL_UART_Transmit_DMA(huart, pData, Size);
	if (status==HAL_ERROR)
	{
		printf("\r\nPC USART Transmit HAL_ERROR\r\n");
	}
	else if (status==HAL_BUSY)
	{
		printf("\r\nPC USART Transmit HAL_BUSY\r\n");
	}
	else if (status==HAL_TIMEOUT)
	{
		printf("\r\nPC USART Transmit HAL_TIMEOUT\r\n");
	}
}

/*###############################################################*/
/*###############################################################* USART_From_PC_To_BLE -->*/
/*###############################################################*/
void USART_From_PC_To_BLE(void)
{
		void *p = NULL;
		p = PC_Receive();
		if (p!=NULL)
		{	
			BLE_Transmit(&huart3, p, To_BLE_Tx_Cnt);
			while(huart3.gState != HAL_UART_STATE_READY)
			{
				osDelay(1);
			}	
			free(p);
		}
}
/*###############################################################*/
/*###############################################################* USART_From_BLE_To_PC -->*/
/*###############################################################*/
void USART_From_BLE_To_PC(void)
{
		void *p = NULL;
		p = BLE_Receive();
		if (p!=NULL)
		{	
			PC_Transmit(&huart6, p, To_PC_Tx_Cnt);
			while(huart6.gState != HAL_UART_STATE_READY)
			{
				osDelay(1);
			}	
			free(p);
		}
}


/*###############################################################*/
/*###############################################################* Bluetooth_Hardware_Reset -->*/
/*###############################################################*/
void Bluetooth_Hardware_Reset(void)
{
		printf("MODUL RESET\r\n");
	  HAL_GPIO_WritePin(Bluetooth_Reset_GPIO_Port, Bluetooth_Reset_Pin, GPIO_PIN_SET);
    osDelay(100);
		HAL_GPIO_WritePin(Bluetooth_Key_GPIO_Port, Bluetooth_Key_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(Bluetooth_Reset_GPIO_Port, Bluetooth_Reset_Pin, GPIO_PIN_RESET);
    osDelay(3000);
		printf("MODUL START\r\n");
}





