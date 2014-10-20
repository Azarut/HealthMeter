/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "stm32f30x.h"
#include "osObjects.h"                      // RTOS object definitions
#include "Board_LED.h" 

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
int main (void) {
	
  osKernelInitialize ();                    // initialize CMSIS-RTOS
	SystemInit();
	LED_Initialize();
  tid_Start_job = osThreadCreate (osThread(Start_job), NULL);
	osKernelStart ();                         // start thread execution 
}
