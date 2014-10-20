/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "stm32f30x.h"
#include "osObjects.h"                      // RTOS object definitions
#include "Board_LED.h" 
#include "stm32f30x_usart.h"

USART_InitTypeDef UART_Config;
/*
 * main: initialize and start the system
 */
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

void USART_Ini(void)
{
	UART_Config.USART_BaudRate = 115200;
	UART_Config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_Config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	UART_Config.USART_Parity = USART_Parity_No;
	UART_Config.USART_StopBits = USART_StopBits_1;
	UART_Config.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &UART_Config);
}

int main (void) 
{
  osKernelInitialize ();                    // initialize CMSIS-RTOS
	SystemInit();
	LED_Initialize();
  tid_Start_job = osThreadCreate (osThread(Start_job), NULL);
	tid_Start_job = osThreadCreate (osThread(Measure_job), NULL);
	osKernelStart ();                         // start thread execution 
}

