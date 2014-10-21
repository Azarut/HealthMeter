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
// USART Transmitter buffer
#define TX_BUFFER_SIZE 350
volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_wr_index=0,tx_rd_index=0;
volatile uint16_t tx_counter=0;

void USART1_IRQHandler(void) 
{ uint8_t cnt = 0;
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
    
    RX_buffer[cnt++] = USART_ReceiveData(USART1);
    

  }
	if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
  {   
		if (tx_counter)
		{
		 --tx_counter;
		 USART_SendData(USART1,tx_buffer[tx_rd_index++]);
		 if (tx_rd_index == TX_BUFFER_SIZE) tx_rd_index=0;
		}
		else
		{
		  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);                  
		}
   }
}

void put_char(uint8_t c)
{
while (tx_counter == TX_BUFFER_SIZE);
USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
if (tx_counter || (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET))
   {
   tx_buffer[tx_wr_index++]=c;
   if (tx_wr_index == TX_BUFFER_SIZE) tx_wr_index=0;
   ++tx_counter;
         USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
   }
else
   USART_SendData(USART1,c);

}

void Start_job (void const *argument)
{
 while(1)
 {
	 USART_SendData(USART1,0x55);
	 LED_On(5);
	 osDelay(1000);
	 LED_Off(5);
	 osDelay(1000);
 }
}

void Measure_job (void const *argument)
{
 while(1)
 {
	
	 LED_On(4);
	 osDelay(100);
	 LED_Off(4);
	 osDelay(100);
 }
}


/*********************НАСТРОЙКА USART1************************/
void USART_Ini(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*Включаем тактирование USART1*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
	
  USART_ITConfig(USART1,USART_IT_PE  ,ENABLE);
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
  USART_ITConfig(USART1,USART_IT_ERR ,ENABLE);
 
}

int main (void) 
{
  osKernelInitialize ();                    // инициализация CMSIS-RTOS
	SystemInit();
	LED_Initialize();
	GPIO_init();
	USART_Ini();
  tid_Start_job = osThreadCreate (osThread(Start_job), NULL);
	tid_Measure_job = osThreadCreate (osThread(Measure_job), NULL);
	osKernelStart ();                         // запуск операционки
}

void GPIO_init(void)
{

}
