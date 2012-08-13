#include <main.h>

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>




void vTaskLED1(void *pvParameters) {

        for (;;) {
                GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET);
                vTaskDelay(20 / portTICK_RATE_MS);
                GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET);
                vTaskDelay(980 / portTICK_RATE_MS);
        }

}

void vTaskLED2(void *pvParameters) {
	vTaskDelay(1000 / portTICK_RATE_MS);


        for (;;) {
                GPIO_SetBits(GPIOB, GPIO_Pin_1);

        	vTaskDelay(200 / portTICK_RATE_MS);

                GPIO_ResetBits(GPIOB, GPIO_Pin_1);

            vTaskDelay(10 / portTICK_RATE_MS);
        }

}

void  Init_GPIOs (void)
{
  /* GPIO, EXTI and NVIC Init structure declaration */
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  /* Configure the LED_pin as output push-pull for LD3 & LD4 usage*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* Force a low level on LEDs*/
  GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET);


}

int main(void) {

	SystemInit();

	
	Init_GPIOs();

	xTaskCreate(vTaskLED1, ( signed char * ) "LED1", configMINIMAL_STACK_SIZE, NULL, 1, ( xTaskHandle * ) NULL);
	//xTaskCreate(vTaskLED2, ( signed char * ) "LED2", configMINIMAL_STACK_SIZE, NULL, 1, ( xTaskHandle * ) NULL);

	vTaskStartScheduler();

	

	while (1) {
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}


void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	volatile uint16_t i;

	while (1) {
		
	}
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	volatile uint16_t i;

	while (1) {
		
	}
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	//__WFI();
}

void vApplicationTickHook(void) {
	

}
