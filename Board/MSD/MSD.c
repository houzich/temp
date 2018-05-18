
#include "ff.h"
//#include "usb_host.h"
#include "MSD.h"
#include "Hub_Define.h"
#include "Hub_Data.h"
#include "Bluetooth_Interface.h"


//extern USBH_HandleTypeDef hUsbHostFS;
//extern USBH_HandleTypeDef  hUSB_Host;
//extern FATFS USBHFatFS;    /* File system object for USBH logical drive */
//extern FIL USBHFile;       /* File object for USBH */
extern char USBHPath[4];   /* USBH logical drive path */



void MSD_File_Write(MCD_File_TypeDef *file)
{
	FRESULT res;
	uint32_t byteswritten;
	//регистрация/дерегистрация рабочей области
	if(f_mount(&file->fat_fs,(TCHAR const*)USBHPath,0)!=FR_OK)
	{
		Error_Handler();
	}
	else
	{
		// создаем файл 
		if(f_open(&file->file,(void*)file->info.fname,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK)
		{
			Error_Handler();
		}
		else
		{
			res = f_write(&file->file, file->data, file->info.fsize,(void*)&byteswritten);
			if((byteswritten==0)||(res!=FR_OK))
			{
				Error_Handler();
			}
			else
			{
				f_close(&file->file);			
				HUB_DEBUG_MSD(("FILE WRITE SUCCESS"));
			}
		}
	}
}

void MSD_File_Read(MCD_File_TypeDef *file)
{
	if(f_mount(&file->fat_fs,(TCHAR const*)USBHPath,0)!=FR_OK)
	{
		Error_Handler();
	}
	else
	{
		if(f_open(&file->file,(void*)file->info.fname,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK)
		{
			Error_Handler();
		}
		else
		{
			Read_Long_File(file);
			f_close(&file->fat_fs);
		}
	}		
}


void MSD_Read_Dir(void)
{

	
	
}

FRESULT Read_Long_File(MCD_File_TypeDef *file)
{
	FRESULT res;
	uint32_t bytesread;
  uint32_t sector_size=0;
  uint64_t read_pointer=0;
  uint64_t f_size = file->fat_fs.fsize;

  HUB_DEBUG_MSD(("fsize: %lu\r\n",(unsigned long)f_size));
  do
  {
		//#define _MIN_SS    512  /* 512, 1024, 2048 or 4096 */
		//#define _MAX_SS    4096  /* 512, 1024, 2048 or 4096 */
    if(f_size < _MAX_SS)
    {
      sector_size=f_size;
    }
    else
    {
      sector_size = _MAX_SS;
    }
    f_size-=sector_size;
		
    f_lseek(&file->file,read_pointer);
		res = f_read(&file->file, file->data, sector_size, (void*)&bytesread);
		if((bytesread==0)||(res!=FR_OK))
		{
			Error_Handler();
		}
		Bluetooth_Send_Data();
    read_pointer+=sector_size;
  }
  while(f_size>0);

  return FR_OK;
}








