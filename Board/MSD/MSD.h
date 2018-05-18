
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MSD_H__
#define __MSD_H__

/* Includes ------------------------------------------------------------------*/
#include "integer.h"	/* Basic integer types */
#include "ffconf.h"		/* FatFs configuration options */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* File information structure for MSD */
typedef struct {
	FILINFO	info;
	FIL file;       		/* File object for USBH */
	FATFS fat_fs;    		/* File system object for USBH logical drive */
	void *data; 				//указатель на данные
	uint32_t data_size; //размер буфера
} MCD_File_TypeDef;


FRESULT Read_Long_File(MCD_File_TypeDef *file);
#endif /* __MSD_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
