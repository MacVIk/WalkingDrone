#include "stm32f1xx.h"
#include "system_stm32f1xx.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"

#include <stdio.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/*
 *   System Clock Configuration
 *   The system Clock is configured as follow :
 *   System Clock source            = PLL (HSE)
 *   SYSCLK(Hz)                     = 72000000
 *   HCLK(Hz)                       = 72000000
 *   AHB Prescaler                  = 1
 *   APB1 Prescaler                 = 2
 *   APB2 Prescaler                 = 1
 *   HSE Frequency(Hz)              = 8000000
 *   PLL Prescaler                  = 1
 *   PLL Multiplicator              = 9
 *   VDD(V)                         = 3.3
 *   Flash Latency(WS)              = 2
 */

/* Demo features */
void set_led_pin();
void run(void *pvParameters);

/*
 * Do not delete this function It provides
 * The correct System Clock settings
 */
void system_clock_config()
{
    /* Enable HSE oscillator */
    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady());

    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    /* Set HSE as source for PLL. Enable PLL */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();
    while (!LL_RCC_PLL_IsReady());

    /* Set AHB (system bus) clock frequency */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* SysClk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    /* Set APB1 and APB2 clock frequency */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

    /* Update CMSIS variable */
    SystemCoreClock = 72000000;
}


int main(void) {

    /* Set described options*/
    system_clock_config();

    /* Set GPIO for LED*/
    set_led_pin();

    /* Task create with a "run" function inside */
    xTaskCreate(run, "run", 64, NULL, 1, NULL);

    /* Start freertos */
    vTaskStartScheduler();

    return 1;
}

void run(void *pvParameters)
{
    /* Infinite circle with a LED flashing */
    while (1) {
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
        vTaskDelay(1000);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
        vTaskDelay(1000);
    }
}

void set_led_pin()
{
    /* Set port clocking */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

    /* User settings */
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
}

