#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "HC0506_USART.h"



/**
  * @brief  Definitions for error constants.
  */
typedef enum {
  BLUETOOTH_ERROR_OK = 0x00U,          /**  */
  BLUETOOTH_ERROR_INIT_FAIL = 0x01U,   /** 	*/
	BLUETOOTH_ERROR_INIT_MEMORY= 0x02U,  /** 	*/
} Bluetooth_ErrorTypeDef;

//#define Bluetooth_ERR_IS_FATAL(e) ((e) <= Bluetooth_ErrorTypeDef.ERR_ABRT)

/**
  * @brief  Definitions for state constants.
  */
typedef enum {
  BLUETOOTH_STATE_RESET = 0x00U,        	/*!<                         										*/
  BLUETOOTH_STATE_BUSY_RESET = 0x01U,   	/*!<      							*/
	BLUETOOTH_STATE_AFTER_RESET = 0x02U,   	/*!<     							*/
//  BLUETOOTH_STATE_PERIPH_READY = 0x02U, /*!<    */
  BLUETOOTH_STATE_READY = 0x03U,        	/*!<                				*/
  BLUETOOTH_STATE_BUSY_INIT = 0x04U,    	/*!<           */
  BLUETOOTH_STATE_ERROR = 0x06U,        	/*!<                       */
//  BLUETOOTH_STATE_ABORT = 0x07U,        /*!<                  */
//  BLUETOOTH_STATE_DISCONNECTED = 0x08U, /*!< 	                               						*/
//  BLUETOOTH_STATE_CONNECTED = 0x09U,    /*!< 	                               						*/
//  BLUETOOTH_STATE_IDLE = 0x19U,         /*!< 	                               						*/
} Bluetooth_StateTypeDef;

/** 
  * @brief  command  
  */
typedef struct
{
  char 		*command;
  char 		*response_as_ok;
  uint16_t timeout_before_response;
  uint16_t timeout_between_packet;
  uint16_t delay_after_response;
} Bluetooth_CommandTypeDef;

/** 
  * @brief  command + flag skip  
  */
typedef struct
{
  const Bluetooth_CommandTypeDef 		*at;
  bool  						 			   skip;
} Bluetooth_HandleCommandTypeDef;		

/** 
  * @brief  Bluetooth session 
  */
typedef const struct {
    Bluetooth_HandleCommandTypeDef *(*coommands)[]; //указатель на массив указателей на структуры Bluetooth_HandleCommandTypeDef
    uint32_t cmds_num;
} Bluetooth_SessionTypeDef;

/** 
  * @brief  Bluetooth handle Structure definition  
  */
typedef struct __Bluetooth_HandleTypeDef
{
  __IO Bluetooth_StateTypeDef state;   					/*!< Bluetooth текушее состояние            					        */
  __IO Bluetooth_ErrorTypeDef error; 	 					/*!< Bluetooth код ошибки                         */
  UART_HandleTypeDef *usart;     								/*!< Bluetooth usart            */
  uint8_t *tx_buff;           	 								/*!< указатель на буфер для отправки      */
  uint16_t tx_buff_size;         								/*!< размер буфера отправки                   */
  uint8_t *rx_buff;           	 								/*!< указатель на буфер приема      */
  uint16_t rx_buff_size;         								/*!< размер буфера приема                   */
  const char *name;     												/*!<                      									*/
  const char *password; 												/*!<                      									*/
  const struct link_hc0506_callbacks *link_cb;	/*!< ссылка на структуру низкоуровневых функций        */
	Bluetooth_SessionTypeDef *curr_session;							/*!< текущая сессия												*/
	Bluetooth_HandleCommandTypeDef *curr_command;				/*!< текущая команда												*/
} Bluetooth_HandleTypeDef;

extern Bluetooth_HandleTypeDef hblueth;

#define Bluetooth_OK_Str "OK"
#define Bluetth_Send_Data(x) printf x

void Bluetooth_Init(void const *argument);
void Bluetooth_Config_Reset_State(Bluetooth_HandleTypeDef *blueth, UART_HandleTypeDef *huart);
void Bluetooth_Hardware_Reset_And_Start(Bluetooth_HandleTypeDef *blueth, UART_HandleTypeDef *huart);
void Bluetooth_Set_Command_Mode(Bluetooth_HandleTypeDef *blueth);
void Bluetooth_Set_Data_Mode(Bluetooth_HandleTypeDef *blueth);


#define BLE_BUFFER_SIZE 100
extern uint8_t BLE_Rx_Buff[];
extern uint8_t BLE_Tx_Buff[];
extern uint8_t PC_Rx_Buff[];
extern uint8_t PC_Tx_Buff[];


void *BLE_Receive(void);
void *PC_Receive(void);
void BLE_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void PC_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void Rx_DMA_Reset(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size);
void USART_From_PC_To_BLE(void);
void USART_From_BLE_To_PC(void);
void Bluetooth_Hardware_Reset(void);
#endif /* __BLUETOOTH_H */
