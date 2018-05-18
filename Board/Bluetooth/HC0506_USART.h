
#ifndef _HC0506_USART_H_
#define _HC0506_USART_H_
/*
 * Структура содержит адреса процедур протокола более низкого уровня соединения.
 * 
 */
struct link_hc0506_callbacks {
void 	(*transmit)(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void 	(*receive)(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
int 	(*wait_receive_data)(UART_HandleTypeDef *huart, uint16_t size, int timeout_before_response, int timeout_between_packet);
int 	(*receive_timeout)(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size, int timeout_before_response, int timeout_between_packet);
void 	(*rx_dma_reset)(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size);
int 	(*get_rx_count)(UART_HandleTypeDef *huart, uint16_t Size);
};
const struct link_hc0506_callbacks* HC0506_Get_Link_Callbacks(void);
void HC0506_USART_Change_BaudRate(UART_HandleTypeDef *huart, uint32_t BaudRate);
#endif /*_HC0506_USART_H_*/

