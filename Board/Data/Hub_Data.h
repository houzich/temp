
#ifndef __HUB_DATA_H__
#define __HUB_DATA_H__
#include <stdint.h>
#include "cmsis_os.h"
#include "lwip/ip_addr.h"
#include "Hub_Define.h"


extern uint8_t Buff_Bluetooth_Rx[BUF_RX_SIZE_BLUETOOTH];
extern uint8_t Buff_Bluetooth_Tx[BUF_TX_SIZE_BLUETOOTH];


//Тестовые
extern char temp_name[100];
extern char temp_password[100];
extern uint32_t fl_start_bluetooth_connect;
extern uint32_t fl_end_bluetooth_connect;
extern char for_test_tx_data[2048];


#endif /* __HUB_DATA_H__ */
