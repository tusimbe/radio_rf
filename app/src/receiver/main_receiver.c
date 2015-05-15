/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 22/02/2015 11:44:43
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "user_diskio.h" /* defines USER_Driver as external */
#include "ppm_decoder.h"
#include "ppm_encoder.h"
#include "radio.h"
#include "telemetry.h"
#include <stdio.h>
#include <string.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;

uint8_t retUSER;    /* Return value for USER */
char USER_Path[4];  /* USER logical drive path */
uint16_t last_edge;
uint16_t width;
uint16_t ppm_pulse_seqence[20];
uint16_t *p_ppm_pulse_seqence;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void uart1_send(uint8_t *buf, uint32_t len);
void _init (void);
void SystemClock_Config(void);
static void MX_SPI1_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void HAL_TIM_IC_OverFlowCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_UpCallback(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    //MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM8_Init();

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    __enable_irq();

    (void)telemetry_init();
    
    (void)radio_host_init();

    printf("system init ok, create tasks ...\r\n");

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    printf("tasks hava created, os starting ...\r\n");
    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    printf("scheduler has been started ...\r\n");

    /* USER CODE BEGIN 3 */
    /* Infinite loop */
    while (1)
    {
        
    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    __HAL_RCC_AFIO_CLK_ENABLE();

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Pinout Configuration
*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* GPIO Ports Clock Enable */
    __GPIOA_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();

    /* LED 0 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    /* NRF_IRQ */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* NRF_CE */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* NRF_CS */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, 
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_NVIC_SetPriority(EXTI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    
    return;
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&hspi1);

    __HAL_SPI_ENABLE(&hspi1);
}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;  /* 1MHz */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xffff;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    HAL_NVIC_SetPriority(TIM3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 1);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_TIM_Base_Start_IT(&htim3);
}

/* TIM8 init function */
void MX_TIM8_Init(void)
{
    PPM_ENCODER ppm;
    uint16_t ppm_last_low_width;
    uint16_t i;

    radio_get_pload((uint8_t*)ppm.channels);
    ppm.valid_chan_num = 8;
    ppm.stop_pulse_width = PPM_STOP_PULSE_WIDTH;
    ppm_encoder(&ppm, ppm_pulse_seqence, &ppm_last_low_width);
    p_ppm_pulse_seqence = &ppm_pulse_seqence[0];

    for (i = 0; i < 16; i++)
    {
        printf("encode ppm chan %02d PWM %04d\r\n", i, ppm_pulse_seqence[i]);
    }

    printf("ppm_last_low_width %d\r\n", ppm_last_low_width);


    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 72 - 1;  /* 1MHz  */
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = *p_ppm_pulse_seqence;
    p_ppm_pulse_seqence++;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim8);

    htim8.Instance->CCR2 = 1000;
    htim8.Instance->CR1 &= ~TIM_CR1_CEN;
    
    htim8.Instance->CCER = TIM_CCER_CC1E;
    htim8.Instance->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2PE;
    htim8.Instance->CCR1 = PPM_STOP_PULSE_WIDTH;
    htim8.Instance->BDTR = TIM_BDTR_MOE;
    htim8.Instance->EGR = 1;
    htim8.Instance->DIER = TIM_DIER_UDE;

    htim8.Instance->SR &= ~TIM_SR_UIF;
    htim8.Instance->SR &= ~TIM_SR_CC2IF;
    htim8.Instance->DIER |= TIM_DIER_CC2IE;
    htim8.Instance->DIER |= TIM_DIER_UIE;
    
    htim8.Instance->CR1 = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM8_UP_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY - 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

extern uint32_t dbg_exit1_int_cnt;
extern uint32_t dbg_tim3_int_cnt;
extern uint32_t dbg_int_max_rt;
extern uint32_t dbg_int_rx_dr;
extern uint32_t dbg_int_tx_ds;
extern uint8_t  dbg_irq_flag;
extern uint16_t dbg_rx_period;
extern uint32_t heartbeat_cnt;
extern uint8_t gzll_chm_get_current_rx_channel(void);

void StartDefaultTask(void const * argument)
{
    argument = argument;
    /*## FatFS: Link the USER driver ###########################*/
    retUSER = FATFS_LinkDriver(&USER_Driver, USER_Path);

    /* USER CODE BEGIN 5 */
    osDelay(1);

    /* Infinite loop */
    for(;;)
    {
        osDelay(1000);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

        /*
        printf("eint %d, tim3:%d, max:%d, tx:%d, rx:%d, tf:0x%x, rf:%d\r\n", 
        dbg_exit1_int_cnt, dbg_tim3_int_cnt, dbg_int_max_rt, dbg_int_tx_ds,
        dbg_int_rx_dr, dbg_irq_flag, gzll_chm_get_current_rx_channel());

        printf("rx poried:%d, int_status:%d\r\n", dbg_rx_period, __HAL_GPIO_EXTI_READ(GPIO_PIN_1));
        */
        printf("hb:%d\r\n", heartbeat_cnt);
    }

    /* USER CODE END 5 */ 

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        htim->Instance->CCER ^= TIM_CCER_CC1P;
        ppm_decode(htim->Instance->CCR1, 0);
    }
    return;
}

void HAL_TIM_IC_OverFlowCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        ppm_decode(htim->Instance->CCR1, PPM_DECODER_OVERFLOW_ERROR);
    }
}

void HAL_TIM_UpCallback(TIM_HandleTypeDef *htim)
{
    PPM_ENCODER ppm;
    uint16_t dummy;
    
    if (htim->Instance == TIM8)
    {
        //printf("T8 UP\r\n");
        #if 1
        __HAL_TIM_CLEAR_IT(htim, TIM_SR_UIF);
        htim->Instance->ARR = *p_ppm_pulse_seqence;
        p_ppm_pulse_seqence++;
        if ((*p_ppm_pulse_seqence) == 0)
        {
            radio_get_pload((uint8_t*)ppm.channels);
            ppm.stop_pulse_width = PPM_STOP_PULSE_WIDTH;
            ppm.valid_chan_num = 8;
            ppm_encoder(&ppm, ppm_pulse_seqence, &dummy);

            htim->Instance->CCR2 = dummy;
            
            //printf("T8En\r\n");
            __HAL_TIM_CLEAR_IT(htim, TIM_SR_CC2IF);
            /* Enable the TIM Capture/Compare 2 interrupt */
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
            //__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
            p_ppm_pulse_seqence = &ppm_pulse_seqence[0];
        }
        #endif
    }
}

void _init (void)
{
  
}

void uart1_send(uint8_t *buf, uint32_t len)
{
     HAL_UART_Transmit(&huart1, buf, len, 5000);
}
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
