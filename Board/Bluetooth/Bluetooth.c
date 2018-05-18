
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

#include "Hub_Data.h"
#include "Bluetooth.h"
#include "HC0506_USART.h"


static void Bluetooth_Hardware_Reset(Bluetooth_HandleTypeDef *Bluetooth);
Bluetooth_HandleTypeDef hblueth;

#define BLUETOOTH_SET_STATE(stat)\
      blueth->state = stat;

#define BLUETOOTH_SET_STATE_ERROR(err)\
      blueth->state = BLUETOOTH_STATE_ERROR;\
      blueth->error = err;
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandAT =
    {
        .command = "AT\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 500,
        .delay_after_response = 500,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandVersionQuestion =
    {
        .command = "AT+VERSION?\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandReset =
    {
        .command = "AT+RESET\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandAddressQuestion =
    {
        .command = "AT+ADDR?\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandNameQuestion =
    {
        .command = "AT+NAME?\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandPasswordQuestion =
    {
        .command = "AT+PSWD?\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandSlaveModeQuestion =
    {
        .command = "AT+ROLE?\r\n",
        .response_as_ok = "ROLE:0",
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandSlaveMode =
    {
        .command = "AT+ROLE=0\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
		
/*1*/
static const Bluetooth_CommandTypeDef BluetoothCommandBoudrate115200 =
    {
        .command = "AT+UART=38400,0,0\r\n",
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 500,
        .delay_after_response = 5000,
};

/*1*/
//эти команды в оперативной памяти размещаем, чтобы можно было менять имя модуля и пароль
//поле command будет заполняться входе инициализации и так же команде пользователя
static Bluetooth_CommandTypeDef BluetoothCommandName =
    {
        .command = NULL,
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};
/*1*/
static Bluetooth_CommandTypeDef BluetoothCommandPassword =
    {
        .command = NULL,
        .response_as_ok = Bluetooth_OK_Str,
        .timeout_before_response = 3000,
        .timeout_between_packet = 100,
        .delay_after_response = 100,
};



//Содержат в себе ссылку на структуру команды во флэш и флаг skip
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandAT = {&BluetoothCommandAT,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandVersionQuestion = {&BluetoothCommandVersionQuestion,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandReset = {&BluetoothCommandReset,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandAddressQuestion = {&BluetoothCommandAddressQuestion,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandNameQuestion = {&BluetoothCommandNameQuestion,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandPasswordQuestion = {&BluetoothCommandPasswordQuestion,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandSlaveModeQuestion = {&BluetoothCommandSlaveModeQuestion,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandSlaveMode = {&BluetoothCommandSlaveMode,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandBoudrate115200 = {&BluetoothCommandBoudrate115200,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandName = {&BluetoothCommandName,0};
static Bluetooth_HandleCommandTypeDef BluetoothHandleCommandPassword = {&BluetoothCommandPassword,0};

/*2*///Набор комманд сессии инициализации Bluetooth для PPPOЕ
const Bluetooth_HandleCommandTypeDef *BluetoothInitCommandsForSlave[] =
{
          &BluetoothHandleCommandAT,
          &BluetoothHandleCommandVersionQuestion,
          &BluetoothHandleCommandAddressQuestion,
          &BluetoothHandleCommandName,
	        &BluetoothHandleCommandNameQuestion,
          &BluetoothHandleCommandPassword,
          &BluetoothHandleCommandPasswordQuestion,
	        &BluetoothHandleCommandSlaveMode,
	        &BluetoothHandleCommandSlaveModeQuestion,
	        &BluetoothHandleCommandBoudrate115200,
	        &BluetoothHandleCommandReset,
};

/*2*///Ссылка на список команд и их количество
Bluetooth_SessionTypeDef BluetoothInitSessionForPPPoE =
{
  .coommands = (Bluetooth_HandleCommandTypeDef *(*)[])&BluetoothInitCommandsForSlave,
  .cmds_num =   (sizeof(BluetoothInitCommandsForSlave) / sizeof(Bluetooth_HandleCommandTypeDef *))
};

/*###############################################################*/
/*###############################################################* Bluetooth_Response_Debug -->*/
/*###############################################################*/
//Используется только при отладке, чтоб строку в одну линию выводить. Убирает символы переноса коретки и т.п.
static void Bluetooth_Response_Debug(Bluetooth_HandleTypeDef *blueth, char *info)
{
    char *cmd = blueth->curr_command->at->command;
    int cmd_size = strlen(cmd);
    uint8_t *p = malloc(cmd_size);
    if (p == NULL)
        return;
    memset(p, 0, cmd_size);

    for (int i = 0; i < cmd_size; i++)
    {
        if ((cmd[i] != 0x00) && ((cmd[i] < 0x20) || (cmd[i] > 0x7F)))
            p[i] = '.';
        else
            p[i] = cmd[i];
        if (p[i] == '\0')
            break;
    }
    printf("%s [%s]", info, p);
    free(p);
}

/*###############################################################*/
/*###############################################################* Send_Cmd_Wait_Response -->*/
/*###############################################################*/
//высылаем команду, ждем ответа. 
static int Send_Cmd_Wait_Response(Bluetooth_HandleTypeDef *blueth)
{
    const Bluetooth_CommandTypeDef *command = blueth->curr_command->at;
    uint8_t *rx_buff = blueth->rx_buff;
    int len = 0;

    blueth->link_cb->rx_dma_reset(blueth->usart, rx_buff, blueth->rx_buff_size);
    blueth->link_cb->receive(blueth->usart, rx_buff, blueth->rx_buff_size);
    // ** Send command to Bluetooth
    if (command->command != NULL)
    {

        HUB_DEBUG_BLUETOOTH(("\r\nAT COMMAND:%s\r\n", command->command));
			blueth->link_cb->transmit(blueth->usart, (uint8_t *)command->command, strlen(command->command));
    }
    // ** Wait for and check the response

    len = blueth->link_cb->wait_receive_data(blueth->usart, blueth->rx_buff_size, command->timeout_before_response, command->timeout_between_packet);
    if (len > 0)
    {
        for (int i = 0; i < len; i++)
            if ((rx_buff[i] < 0x20) || (rx_buff[i] >= 0x80))
                rx_buff[i] = 0x2e;

        // Check the response
        if (strstr((char *)rx_buff, command->response_as_ok) != NULL)
        {
            HUB_DEBUG_BLUETOOTH(("AT RESPONSE(%i): [%s]\r\n", len, rx_buff));
            return len;
        }
				else if (len == 1)
        {
            HUB_DEBUG_BLUETOOTH(("RESPONSE URC: %i\r\n", rx_buff[0]));
            return 1;
        }
        else
        {
            HUB_DEBUG_BLUETOOTH(("AT BAD RESPONSE(%i): [%s]\r\n", len, rx_buff));
            return 0;
        }
    }
    else
    {
        HUB_DEBUG_BLUETOOTH(("AT: TIMEOUT\r\n"));
        return 0;
    }
}

/*###############################################################*/
/*###############################################################* Enable_Init_Cmd -->*/
/*###############################################################*/
//для всех команд сбрасываем флаг skip
static void Enable_Init_Cmd(Bluetooth_HandleTypeDef *blueth)
{
    Bluetooth_SessionTypeDef *init = blueth->curr_session;
    for (int idx = 0; idx < init->cmds_num; idx++)
    {
      (*(init->coommands))[2]->skip = 0;
    }
}

/*###############################################################*/
/*###############################################################* Fill_Name_Command_Handle -->*/
/*###############################################################*/
//Заполняе структуру для AT+NAME= команды
static char * Fill_Name_Command_Handle(Bluetooth_HandleTypeDef *blueth)
{
    BluetoothCommandName.command = "AT+NAME=\"%s\"\r\n"; //вставляем шаблон
    uint32_t str_size = strlen(blueth->name) + strlen(BluetoothCommandName.command)- sizeof("%s");
    char *p = malloc(str_size);
    if (p == NULL) {
      BLUETOOTH_SET_STATE_ERROR(BLUETOOTH_ERROR_INIT_MEMORY)
      return NULL;
    }
    sprintf(p, BluetoothCommandName.command, blueth->name);
    BluetoothCommandName.command = p;
return p;
}
/*###############################################################*/
/*###############################################################* Fill_Password_Command_Handle -->*/
/*###############################################################*/
//Заполняе структуру для AT+PSWD= команды
static char * Fill_Password_Command_Handle(Bluetooth_HandleTypeDef *blueth)
{
    BluetoothCommandPassword.command = "AT+PSWD=\"%s\"\r\n"; //вставляем шаблон
    uint32_t str_size = strlen(blueth->password) + strlen(BluetoothCommandPassword.command)- sizeof("%s");
    char *p = malloc(str_size);
    if (p == NULL) {
      BLUETOOTH_SET_STATE_ERROR(BLUETOOTH_ERROR_INIT_MEMORY)
      return NULL;
    }
    sprintf(p, BluetoothCommandPassword.command, blueth->password);
    BluetoothCommandPassword.command = p;
return p;
}
/*###############################################################*/
/*###############################################################* Bluetooth_Init -->*/
/*###############################################################*/
void Bluetooth_Init(void const *argument)
{
	Bluetooth_HandleTypeDef *blueth = (Bluetooth_HandleTypeDef *)argument; //передаем сюда структуру Bluetooth
/*3*/Bluetooth_HandleCommandTypeDef *(*init_commands)[] = blueth->curr_session->coommands; //передаем список команд текущей сессии инициализации
	uint32_t init_cmds_num = blueth->curr_session->cmds_num; //количество команд в текущей сессии инициализации
	uint32_t cmd_count = 0; //счетчик комманд
	uint32_t fail_number = 0; //количество отказов или попыток выслать команду и получить положительный ответ
	uint32_t err = 0;
	
  BLUETOOTH_SET_STATE(BLUETOOTH_STATE_BUSY_INIT) //ставим статус "заняты инициализацией"
	Bluetooth_Hardware_Reset(blueth); //сбрасываем ножкой
	
	char *cmd_apn = Fill_Name_Command_Handle(blueth); //заполнаяем структуру команды NAME, может вернуть ошибку выделения памяти (мало ли, чего не бывает)
  if(cmd_apn==NULL) goto exit;
	cmd_apn = Fill_Password_Command_Handle(blueth); //заполнаяем структуру команды PASSWORD
  if(cmd_apn==NULL) goto exit;
	
/*4*/Enable_Init_Cmd(blueth); //сбрасываем флаг skip на всех командах
  HUB_DEBUG_BLUETOOTH(("Bluetooth initialization start\r\n"));

	// проходимся по всем командам
    while (cmd_count < init_cmds_num) 
    {
			Bluetooth_HandleCommandTypeDef *cmd = (*(init_commands))[cmd_count]; //записывыем сюда структуру текущей команды, для читабельности
        blueth->curr_command = cmd;
			if (cmd->skip) //если уже посылали такую
        {
#ifdef HUB_DEBUG_BLUETOOTH
					Bluetooth_Response_Debug(blueth, "Skip command:"); //используется для отладки
#endif
            cmd_count++;
            continue;
        }
				
				err = Send_Cmd_Wait_Response(blueth);
        if ((err == 0)||(err == 1))
        {
					//Нет ответа или не такой как ожидалось, если превышенно количество попытак устанавливаем статус в ошибку и выходим
            HUB_DEBUG_BLUETOOTH(("Wrong response, restarting...\r\n"));
            fail_number++;
            if (fail_number > 20){
              BLUETOOTH_SET_STATE_ERROR(BLUETOOTH_ERROR_INIT_FAIL);
              goto exit;
            }


						 //Если не получили ответа на команду AT, еще раз сбрасываем аппаратно
						//а если URC то не сбрасываем
            if((cmd->at==&BluetoothCommandAT)&&(err == 0)) Bluetooth_Hardware_Reset(blueth);
            cmd_count = 0;
            continue;
        }

				//Получили положительный ответ на команду Bod115200, меняем скорость усарта
        //if(cmd->at==&BluetoothCommandBoudrate115200) HC0506_USART_Change_BaudRate(blueth->usart, 115200);
				
        if (cmd->at->delay_after_response > 0) //если установили задержку после команды, чтоб модуль успел прийти в себя (особенно актуально после команды сброса и еще нескольких) 
            osDelay(cmd->at->delay_after_response);
        cmd->skip = 1;
        // Next command
        cmd_count++;
    }
		Bluetooth_Set_Data_Mode(blueth);
    HUB_DEBUG_BLUETOOTH(("Bluetooth initialized.\r\n"));
		BLUETOOTH_SET_STATE(BLUETOOTH_STATE_READY);
exit:
    if(cmd_apn != NULL) free(cmd_apn);
    for (;;)
    {
        /* Delete the Init Thread */
        osThreadTerminate(NULL);
    }
}
/*###############################################################*/
/*###############################################################* Bluetooth_Config_Reset_State -->*/
/*###############################################################*/
void Bluetooth_Config_Reset_State(Bluetooth_HandleTypeDef *blueth, UART_HandleTypeDef *huart)
{
    blueth->state = BLUETOOTH_STATE_RESET;                    											/*!< Bluetooth состояние                     */
		blueth->error = BLUETOOTH_ERROR_OK;                       											/*!< Bluetooth код ошибки                         */
    blueth->usart = huart;                              														/*!< передаем сюда usart для блютуза           */
    blueth->tx_buff = Buff_Bluetooth_Tx;                      											/*!< буффер Tx       */
    blueth->tx_buff_size = BUF_TX_SIZE_BLUETOOTH;             											/*!< размер                   */
    blueth->rx_buff = Buff_Bluetooth_Rx;                      											/*!< буфер Rx       */
    blueth->rx_buff_size = BUF_RX_SIZE_BLUETOOTH;             											/*!< размер                   */
		blueth->name = temp_name;         																							/*!< строка имени модуля                     			  */
    blueth->password = temp_password; 																							/*!< стро адреса                       */
    blueth->link_cb = HC0506_Get_Link_Callbacks();      														/*!< L        */
    blueth->curr_session = &BluetoothInitSessionForPPPoE;              							/*!< Сессия для использования РРР               			*/
		blueth->curr_command = (*(BluetoothInitSessionForPPPoE.coommands))[0];    			/*!< первая команда настройки РРР           		            */
}

/*###############################################################*/
/*###############################################################* Bluetooth_Hardware_Reset_And_Start -->*/
/*###############################################################*/
//Делаем аппаратный сброс, вызываем поток для инициализии модема
void Bluetooth_Hardware_Reset_And_Start(Bluetooth_HandleTypeDef *blueth, UART_HandleTypeDef *huart)
{
    HUB_DEBUG_BLUETOOTH(("\r\nMODUL Start\r\n"));
    Bluetooth_Config_Reset_State(blueth, huart); //аппаратно сбрасываем

    osThreadDef(bluethTask, Bluetooth_Init, BLUETOOTH_INIT_THREAD_PRIO, 0, BLUETOOTH_INIT_THREAD_STACKSIZE);
    osThreadCreate(osThread(bluethTask), blueth);
    while (blueth->state != BLUETOOTH_STATE_READY) //ждем финала
    {
			if(blueth->state == BLUETOOTH_STATE_ERROR){ //обрабатываем ошибки
          HUB_DEBUG_BLUETOOTH(("\r\nBluetooth INIT ERROR\r\n"));

          switch(blueth->error)
          {
            case  BLUETOOTH_ERROR_INIT_FAIL:     Bluetooth_Config_Reset_State(blueth, huart); break;
            default:                       Bluetooth_Config_Reset_State(blueth, huart); break;
          }
        }
        osDelay(1);
    }
}
/*###############################################################*/
/*###############################################################* Bluetooth_Hardware_Reset -->*/
/*###############################################################*/
static void Bluetooth_Hardware_Reset(Bluetooth_HandleTypeDef *blueth)
{
		HUB_DEBUG_BLUETOOTH(("MODUL RESET\r\n"));
		BLUETOOTH_SET_STATE(BLUETOOTH_STATE_BUSY_RESET)
	  HAL_GPIO_WritePin(Bluetooth_Reset_GPIO_Port, Bluetooth_Reset_Pin, GPIO_PIN_SET);
    osDelay(100);
		HAL_GPIO_WritePin(Bluetooth_Key_GPIO_Port, Bluetooth_Key_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(Bluetooth_Reset_GPIO_Port, Bluetooth_Reset_Pin, GPIO_PIN_RESET);
    osDelay(3000);
		BLUETOOTH_SET_STATE(BLUETOOTH_STATE_AFTER_RESET)
}
/*###############################################################*/
/*###############################################################* Bluetooth_Set_Command_Mode -->*/
/*###############################################################*/
void Bluetooth_Set_Command_Mode(Bluetooth_HandleTypeDef *blueth)
{
		HUB_DEBUG_BLUETOOTH(("MODUL SET COMMAND MODE\r\n"));
		HAL_GPIO_WritePin(Bluetooth_Key_GPIO_Port, Bluetooth_Key_Pin, GPIO_PIN_SET);
    osDelay(100);
}
/*###############################################################*/
/*###############################################################* Bluetooth_Set_Data_Mode -->*/
/*###############################################################*/
void Bluetooth_Set_Data_Mode(Bluetooth_HandleTypeDef *blueth)
{
		HUB_DEBUG_BLUETOOTH(("MODUL SET DATA MODE\r\n"));
		HAL_GPIO_WritePin(Bluetooth_Key_GPIO_Port, Bluetooth_Key_Pin, GPIO_PIN_RESET);
    osDelay(1000);
}
