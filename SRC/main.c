/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
#include <string.h>
#define osObjectsPublic                     // define objects in main module
#include "stm32f30x.h"
#include "osObjects.h"                      // RTOS object definitions
#include "Board_LED.h" 
#include "stm32f30x_usart.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_gpio.h"

#define GREEN 7
#define RED 	5

volatile uint8_t led_state = 7;
#define WAIT_STATE 	0
#define SND_GOOD	 	1
#define SRV_ERROR 	2
#define SRV_DOWN	 	3
#define ALL_ERROR	 	4
#define RPT_GOOD	 	5
#define RPT_BAD		 	6
#define SND_BAD		 	7
#define DEVICE_BAD	8

void GPIO_init(void);
void put_char(char c);

/*      AD переменные          */
int16_t  Version = 0x0003;
int32_t  TransmitterID = 123456789;
int16_t  MessageType = 3;
int16_t  DataType = 3082;
uint32_t DeltaTime = 2000;
uint16_t SystolicPressure = 141;
uint16_t DiastolicPressure = 97;
uint16_t Pulse = 88;
/*******************************/

/*Cкорость мигания светодиодов */
uint16_t WAIT = 5000;
uint16_t SEND = 333;
uint16_t blink_speed = 5000;
/*******************************/

/*  Флаги необходимых действий */
uint8_t need_setup = 1;
uint8_t need_repeat = 0;
/*******************************/

/*    Флаги новых событий      */
uint8_t sms_flag = 0;						
uint8_t chek_flag = 0;
uint8_t new_sms_flag = 0;
uint8_t new_measure_flag = 0;
/*******************************/

uint8_t repeat_cnt = 0;
uint8_t err_cnt = 0;
/*    АТ-комманды SIM800       */
uint8_t ECHO[5] = "ATE0\r"; 
uint8_t SET_SMS_OP_1[10] = "AT+CMGF=1\r"; 
uint8_t SET_SMS_OP_2[15] = "AT+CSCS= \"GSM\"\r"; 
uint8_t READ_SMS[10] = "AT+CMGR=1\r"; 
uint8_t DEL_SMS[19] = "AT+CMGDA=\"DEL ALL\"\r";
uint8_t APN[19] = "AT+CSTT=\"internet\"\r"; 
uint8_t CHK_CONNECT[9] =  "AT+CIICR\r";
uint8_t CHK_IP[9] = "AT+CIFSR\r";
uint8_t SRV_CONNECT[42] = "AT+CIPSTART=\"TCP\",\"82.204.241.138\",\"8081\"\r";
uint8_t SEND_DATA[11] = "AT+CIPSEND\r";
uint8_t CHK_GPRS[10] = "AT+CGATT?\r";
uint8_t GPRS_CONNECT[11] = "AT+CGATT=1\r";
uint8_t END_LINE[1] = {0x1A};
/*******************************/


/*       Буферы данных         */
uint8_t RX_buffer[255] = {0};
uint8_t AD_buffer[255] = {0};
uint8_t SMS_buffer[64] = {0};
uint8_t check_buffer[6] = {0};
uint8_t MEASURE[22] = {0};
uint8_t aMeasBuffer[6];
/*******************************/

#define CRC16_INITIAL_REMAINDER 0x0000
#define CRC16_FINAL_XOR_VALUE   0x0000
#define CRC16_WIDTH (8 * sizeof(uint16_t))
uint16_t CRC_calc = 0;

static const uint16_t crcTable[256] =
    { 0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 };

uint16_t crc16(const void * const message, const uint16_t nBytes) {
    uint8_t  data;
    uint16_t byte, remainder = CRC16_INITIAL_REMAINDER;

    for (byte = 0; byte < nBytes; ++byte) {
        data = ((const uint8_t * const) message)[byte] ^ (remainder >> (CRC16_WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    return (remainder ^ CRC16_FINAL_XOR_VALUE);
}


void USART2_IRQHandler(void) 
{ uint8_t uart_data_AD = 0; static uint8_t AD_Cnt = 0; 
// Recognizing the interrupt event
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  if(USART_GetFlagStatus(USART2,USART_FLAG_ORE))
  {
    USART_ClearFlag(USART2,USART_FLAG_ORE);
    // Overrun Error
  }
  if(USART_GetFlagStatus(USART2,USART_FLAG_FE))
  {
    USART_ClearFlag(USART2,USART_FLAG_FE);
    // Framing Error
  }
  if(USART_GetFlagStatus(USART2,USART_FLAG_NE))
  {
    USART_ClearFlag(USART2,USART_FLAG_NE);
    // Noise Error
  }
  if(USART_GetFlagStatus(USART2,USART_FLAG_PE))
  {
    USART_ClearFlag(USART2,USART_FLAG_PE);
    // Parity Error
  }
  // Push a new data into the receiver buffer
  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
  {
    // Push a new data into the receiver buffer
    uart_data_AD = USART_ReceiveData(USART2);
		if(uart_data_AD == 'U') AD_Cnt = 0;
		AD_buffer[AD_Cnt++] = uart_data_AD;
		if((AD_buffer[1] == 'U') && (AD_Cnt >= 5))
		{
				USART_ITConfig(USART2,USART_IT_PE  ,DISABLE);
				USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
				USART_ITConfig(USART2,USART_IT_ERR ,DISABLE);
				new_measure_flag = 1;			
		}
  }
}

void USART1_IRQHandler(void) 
{ static uint8_t cnt = 0, sec_cnt = 0;
	uint8_t uart_data = 0;
// Recognizing the interrupt event
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE))
  {
    USART_ClearFlag(USART1,USART_FLAG_ORE);
    // Overrun Error
  }
  if(USART_GetFlagStatus(USART1,USART_FLAG_FE))
  {
    USART_ClearFlag(USART1,USART_FLAG_FE);
    // Framing Error
  }
  if(USART_GetFlagStatus(USART1,USART_FLAG_NE))
  {
    USART_ClearFlag(USART1,USART_FLAG_NE);
    // Noise Error
  }
  if(USART_GetFlagStatus(USART1,USART_FLAG_PE))
  {
    USART_ClearFlag(USART1,USART_FLAG_PE);
    // Parity Error
  }
  // Push a new data into the receiver buffer
  if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))
  {
    // Push a new data into the receiver buffer
    uart_data = USART_ReceiveData(USART1);
		if(uart_data == '+')
		{
		  sec_cnt = 0;
			chek_flag = 1;
		}
		
		if((uart_data == '#') && !sms_flag)
		{
			cnt = 0;
			sms_flag = 1;
		}
		else if((uart_data == '#') && sms_flag) sms_flag = 0;
		

		
		if(chek_flag)
		{
		  check_buffer[sec_cnt++] = uart_data;
			if(sec_cnt == 6) 
			{
				chek_flag = 0;
				if(check_buffer[3] == 'T') 
				{
					check_buffer[3] = ' ';
					new_sms_flag = 1;
				}
			}
			
		}
			
		if(sms_flag) 
			SMS_buffer[cnt++] = uart_data;
    else 
			RX_buffer[cnt++] = uart_data;
		if(RX_buffer[3] == 'S') need_setup = 1;
	  if(uart_data == 10)
			cnt = 0;
		if(RX_buffer[0] == 'E')
			err_cnt++;
		else err_cnt = 0;
		if(err_cnt > 3) led_state = RPT_BAD;
  }
}
void send_str(uint8_t string[], uint8_t lenghth) 
{
	uint8_t i=0;
	while(i<lenghth) 
		{
		put_char(string[i]);
		i++;
		}
	//put_char('\r');
	//put_char('\n');
}

void put_char(char c)
{
	 while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	 USART_SendData(USART1,c);

}

void Start_job (void const *argument)
{ 
	while(1)
 { 
	 if(need_setup)
	 {
			led_state = DEVICE_BAD;
			osDelay(7000);
			led_state = SND_GOOD;
			need_setup = 0;
		  send_str(ECHO, 5);
		  osDelay(1000);
		  send_str(SET_SMS_OP_1, 10);
		  osDelay(1000);
		  send_str(SET_SMS_OP_2, 15);;
		  osDelay(1000);
		  send_str(DEL_SMS, 19);
		  osDelay(1000);
		  send_str(CHK_GPRS, 10);
		  osDelay(1000);
	    send_str(GPRS_CONNECT, 11);
		  osDelay(1000);
      send_str(APN, 19);
		  osDelay(1000);
	    send_str(CHK_CONNECT, 9);
		  osDelay(1000);
	    send_str(CHK_IP, 9);
		  osDelay(1000);
	    send_str(SRV_CONNECT, 42);
		  osDelay(3000);
			if(err_cnt < 3) led_state = WAIT_STATE;
	 }
	 osDelay(1000);
 }
}

void Measure_job (void const *argument)
{ static uint8_t dbg_cnt = 0; uint8_t i = 0;
 while(1)
 {
	 if(new_sms_flag)
	 {
	 		new_sms_flag = 0;
		  send_str(READ_SMS, 10);
		  osDelay(100);
		  send_str(DEL_SMS, 19);
		  osDelay(100);
		  WAIT = (SMS_buffer[3]-0x30)*1000 + (SMS_buffer[4]-0x30)*100 + (SMS_buffer[5]-0x30) * 10 + (SMS_buffer[6]-0x30);
		  repeat_cnt = SMS_buffer[1]-0x30;
	 }
	 if(new_measure_flag)
	 {
			led_state = SND_GOOD;
		  new_measure_flag = 0;
			for(i=0;i<=5;i++)
			{
				if(AD_buffer[2*i+1] > 0x040)
					aMeasBuffer[i] = (AD_buffer[2*i+1] - 0x37) << 4;
				else	
					aMeasBuffer[i] = (AD_buffer[2*i+1] - 0x30) << 4;
				
				if(AD_buffer[2*i+2] > 0x040)
					aMeasBuffer[i] += AD_buffer[2*i+2] - 0x37;
				else
					aMeasBuffer[i] += AD_buffer[2*i+2] - 0x30;
			}
		
			MEASURE[15] = aMeasBuffer[1] + aMeasBuffer[2];
			MEASURE[17] = aMeasBuffer[2];
			MEASURE[19] = aMeasBuffer[3];
			CRC_calc = crc16((uint32_t*)MEASURE, 20);
			MEASURE[21] = (uint8_t)CRC_calc;
			MEASURE[20] = CRC_calc >> 8;
		  send_str(SEND_DATA, 11);
			send_str(MEASURE, 22);
      put_char(0x1A);
			USART_ITConfig(USART2,USART_IT_PE  ,ENABLE);
			USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
			USART_ITConfig(USART2,USART_IT_ERR ,ENABLE);
		  led_state = WAIT_STATE;
	 }
	 if(repeat_cnt)
	 {
		 dbg_cnt = repeat_cnt;
		 while(dbg_cnt--)
		 {
				LED_On(4);
				osDelay(50);
				LED_Off(4);
			  osDelay(200);
		 }
	   osDelay(WAIT);
	 }
	 osDelay(1000);
 }
}

void Repeat_job (void const *argument)
{ uint8_t r_cnt = 0;
 while(1)
 {
   if(need_repeat) 
	 {
	   r_cnt = repeat_cnt;
		 while(r_cnt--)
		 {
			 
		 }
	 }
	 osDelay(WAIT);
 }
}

void Blink_job (void const *argument)
{ uint8_t blink_cnt = 0;
 while(1)
 {
	   switch(led_state)
		 {
			 case WAIT_STATE:											 
											 LED_Off(RED);
											 LED_On(GREEN);
											 osDelay(10);
											 LED_Off(GREEN);
											 osDelay(5000);				 
			 break;
			 case SND_GOOD:
											 LED_Off(RED);
											 LED_On(GREEN);
											 osDelay(10);
											 LED_Off(GREEN);
											 osDelay(333);					 
			 break;
			 case SRV_ERROR:
											 LED_Off(GREEN);
											 LED_Off(RED);
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(RED);
											 osDelay(1000);				 
			 break;
			 case SRV_DOWN:
											 LED_Off(GREEN);
											 LED_Off(RED);
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(RED);
											 osDelay(200);
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(RED);
											 osDelay(2000);	
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(RED);
											 osDelay(200);
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(RED);
											 led_state = 	RPT_GOOD;	
											 osDelay(5000);			 
			 break;
			 case ALL_ERROR:
											 LED_Off(GREEN);
											 LED_Off(RED);
											 blink_cnt = 5;
											 while(blink_cnt--)
											 {
													LED_On(RED);
													osDelay(10);
													LED_Off(RED);
													osDelay(500);
											 }	
											 led_state = 	RPT_GOOD;	
											 osDelay(5000);											 
			 break;
			 case RPT_GOOD:
											 LED_On(GREEN);
											 LED_Off(RED);
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(GREEN);
											 LED_Off(RED);
											 osDelay(5000);					 
			 break;
			 case RPT_BAD:
											 LED_Off(GREEN);
											 LED_Off(RED);
											 LED_On(RED);
											 osDelay(10);
											 LED_Off(RED);
											 osDelay(5000);					 
			 break;
			 case SND_BAD:
											 LED_Off(GREEN);
											 LED_Off(RED);
											 blink_cnt = 5;
											 while(blink_cnt--)
											 {
													LED_On(RED);
													osDelay(10);
													LED_Off(RED);
													osDelay(500);
											 }	
											 led_state = 	RPT_BAD;					 
											 osDelay(5000);
			 break;
			 case DEVICE_BAD:
												LED_On(RED);	
												osDelay(1000);			 
			 break;
		 }
 }
}
/*********************НАСТРОЙКА USART1************************/
void USART_Ini(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef UART_Config;
	
	/*Включаем тактирование USART1*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/*Настраиваем порты USART1*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_7);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_7);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);
	
	/*Настраиваем параметры USART1 и запускаем*/
	UART_Config.USART_BaudRate = 115200;
	UART_Config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_Config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	UART_Config.USART_Parity = USART_Parity_No;
	UART_Config.USART_StopBits = USART_StopBits_1;
	UART_Config.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &UART_Config);
	USART_Cmd(USART1, ENABLE);
	
	UART_Config.USART_BaudRate = 9600;
	UART_Config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_Config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	UART_Config.USART_Parity = USART_Parity_No;
	UART_Config.USART_StopBits = USART_StopBits_1;
	UART_Config.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &UART_Config);
	USART_Cmd(USART2, ENABLE);
	
  /*Настраиваем прерывание USART1*/ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
	
  USART_ITConfig(USART1,USART_IT_PE  ,ENABLE);
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
  USART_ITConfig(USART1,USART_IT_ERR ,ENABLE);
	
	USART_ITConfig(USART2,USART_IT_PE  ,ENABLE);
  USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
  USART_ITConfig(USART2,USART_IT_ERR ,ENABLE);
 
}

int main (void) 
{
  osKernelInitialize ();                    // инициализация CMSIS-RTOS
	SystemInit();
	LED_Initialize();
	GPIO_init();
	USART_Ini();
	MEASURE[1] = (uint8_t)Version;
	MEASURE[0] = Version >> 8;
	MEASURE[5] = (uint8_t)TransmitterID;
	MEASURE[4] = TransmitterID >> 8;
	MEASURE[3] = TransmitterID >> 16;
	MEASURE[2] = TransmitterID >> 24;
	MEASURE[7] = (uint8_t)MessageType;
	MEASURE[6] = MessageType >> 8;
	MEASURE[9] = (uint8_t)DataType;
	MEASURE[8] = DataType >> 8;
	MEASURE[13] = (uint8_t)DeltaTime;
	MEASURE[12] = DeltaTime >> 8;
	MEASURE[11] = DeltaTime >> 16;
	MEASURE[10] = DeltaTime >> 24;
  tid_Start_job = osThreadCreate (osThread(Start_job), NULL);
	tid_Measure_job = osThreadCreate (osThread(Measure_job), NULL);
	tid_Repeat_job = osThreadCreate (osThread(Repeat_job), NULL);
	tid_Blink_job = osThreadCreate (osThread(Blink_job), NULL);
	osKernelStart ();                         // запуск операционки
}

void GPIO_init(void)
{

}
