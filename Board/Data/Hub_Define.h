
#ifndef __HUB_DEFINE_H__
#define __HUB_DEFINE_H__

#define inet_addr_my(a, b, c, d) (((uint32_t)a) | ((uint32_t)b << 8) | \
								  ((uint32_t)c << 16) | ((uint32_t)d << 24))


//BLUETOOTH Init
#define BLUETOOTH_INIT_THREAD_STACKSIZE 	512
#define BLUETOOTH_INIT_THREAD_PRIO 				osPriorityAboveNormal
//BLUETOOTH Config
#define CONFIG_BLUETOOTH_APN 			"anton"
#define CONFIG_BLUETOOTH_NAME 		"anton"
#define CONFIG_BLUETOOTH_PASSWORD "anton"
#define BUF_TX_SIZE_BLUETOOTH 		1024
#define BUF_RX_SIZE_BLUETOOTH 		1024

#define HUB_DEBUG_BLUETOOTH(x) printf x
#define HUB_DEBUG_USB(x) printf x
#define HUB_DEBUG_MSD(x) printf x

#endif /* __HUB_DEFINE_H__ */
