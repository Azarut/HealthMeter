/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "stm32f30x.h"
#include "osObjects.h"                      // RTOS object definitions
#include "Board_LED.h" 
#include "stm32f30x_usart.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_gpio.h"

void GPIO_init(void);
USART_InitTypeDef UART_Config;
uint8_t RX_buffer[255] = {0};

void USART1_IRQHandler(void) 
{ uint8_t cnt = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			RX_buffer[cnt]=USART_ReceiveData (USART2);
			cnt++;
		}
}

void Start_job (void const *argument)
{
 while(1)
 {
	
	 osDelay(1000);
 }
}

void Measure_job (void const *argument)
{
 while(1)
 {
	
	 osDelay(1000);
 }
}


/*********************НАСТРОЙКА USART1************************/
void USART_Ini(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Включаем тактирование USART1*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/*Настраиваем порты USART1*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	/*Настраиваем параметры USART1 и запускаем*/
	UART_Config.USART_BaudRate = 115200;
	UART_Config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_Config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	UART_Config.USART_Parity = USART_Parity_No;
	UART_Config.USART_StopBits = USART_StopBits_1;
	UART_Config.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &UART_Config);
	USART_Cmd(USART1, ENABLE); 
  /*Настраиваем прерывание USART1*/ 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
 
}

int main (void) 
{
  osKernelInitialize ();                    // инициализация CMSIS-RTOS
	SystemInit();
	LED_Initialize();
	GPIO_init();
  tid_Start_job = osThreadCreate (osThread(Start_job), NULL);
	tid_Start_job = osThreadCreate (osThread(Measure_job), NULL);
	osKernelStart ();                         // запуск операционки
}

void GPIO_init(void)
{

}
