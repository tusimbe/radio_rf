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
#include "hal_lcd.h"
#include "pcm_decoder.h"
#include "key.h"
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
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
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
    pcm_decoder_init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();

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
    
    key_init();
    __enable_irq();
    (void)radio_device_init();

    printf("system init ok, create tasks ...\r\n");
    
    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
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
    __GPIOB_CLK_ENABLE();

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
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_SET);

    HAL_NVIC_SetPriority(EXTI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* LCD D0-D15 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_All, GPIO_PIN_SET);    

    /* PC6-PC10*/
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, 
        GPIO_PIN_SET);
        
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


void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1; /* 1MHZ */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xffff;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    HAL_TIM_IC_Init(&htim2);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

    HAL_NVIC_SetPriority(TIM2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);

    return;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
extern uint32_t dbg_tim3_int_cnt;
extern uint32_t dbg_exit1_int_cnt;
extern uint32_t dbg_int_max_rt;
extern uint32_t dbg_int_rx_dr;
extern uint32_t dbg_int_tx_ds;
extern uint8_t  dbg_irq_flag;
extern uint16_t dbg_rx_period;
extern uint32_t dbg_tx_retrans;
extern bool volatile gzll_sync_on;
extern uint8_t gzp_system_address[4];
extern bool pairing_ok;
extern uint8_t gzll_chm_get_current_rx_channel(void);

uint8_t menu_idx;
void StartDefaultTask(void const * argument)
{
    uint16_t id = 0;
    argument = argument;
    uint8_t test_addr[4] = {0x11, 0x22, 0x33, 0x44};
    /*## FatFS: Link the USER driver ###########################*/
    retUSER = FATFS_LinkDriver(&USER_Driver, USER_Path);

    /* USER CODE BEGIN 5 */
    osDelay(1);

    LCD_Init();
    LCD_Clear(BLUE);

    POINT_COLOR=RED;
    LCD_ShowString(30,40,200,24,24,"Mini STM32 ^_^");    

    /* enable TIM2 capture interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    /* Infinite loop */
    for(;;)
    {
        osDelay(100);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
        //pcm_channel_show();

        switch(key_scan(0))
        {
            case KEY0_PRES:
                menu_idx++;
                if (menu_idx == 8)
                {
                    menu_idx = 0;
                }

                printf("menu idx %d.\r\n", menu_idx);
                break;
            case KEY1_PRES:
                switch (menu_idx)
                {
                    case 0:
                        pcm_frame_show();
                        break;
                    case 1:
                        pcm_decoder_start();
                        break;
                    case 2:
                        pcm_decoder_stop();
                        break;
                    case 3:
                        pcm_decoder_reset();
                        break;
                    case 4:
                        pcm_dbg_decode_step();
                        break;
                    case 5:
                        pcm_channel_show();
                        break;
                    case 6:
                        pairing_list_entry_show(3);
                        break;
                    case 7:
                        pairing_list_addr_write(3, test_addr);
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
#if 0
        printf("eint %d, tim3:%d, max:%d, tx:%d, rx:%d, tf:0x%x, rf:%d, primask:%d\r\n", 
        dbg_exit1_int_cnt, dbg_tim3_int_cnt, dbg_int_max_rt, dbg_int_tx_ds,
        dbg_int_rx_dr, dbg_irq_flag, gzll_sync_on, __get_PRIMASK());

        //printf("rx poried:%d, isr_max_tick:%d, min_tick:%d, tx_retry:%d\r\n", 
        //dbg_rx_period, dbg_isr_max_tick, dbg_isr_min_tick, dbg_tx_retrans);
        printf("pairing_status:%d, sys_addr:0x%x%x%x%x\r\n", pairing_ok, gzp_system_address[0], 
        gzp_system_address[1], gzp_system_address[2], gzp_system_address[3]);
#endif
    }

    /* USER CODE END 5 */ 

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        //htim->Instance->CCER ^= TIM_CCER_CC1P;
        pcm_decode(htim->Instance->CCR1, 0);
    }
    
    return;
}

void HAL_TIM_IC_OverFlowCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        //pcm_decode(htim->Instance->CCR1, PCM_FRAME_OVERFLOW_ERR);
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
