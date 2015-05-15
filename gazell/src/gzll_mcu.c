
#include "stm32f1xx_hal.h"
#include "gzll.h"

extern TIM_HandleTypeDef htim3;

uint32_t dbg_tim3_int_cnt;
uint32_t dbg_exit1_int_cnt;
uint16_t dbg_rx_period;
uint32_t timer_1ms;
void gzll_set_timer_period(uint16_t period)
{
    dbg_rx_period = period;
    htim3.Instance->ARR = period;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t flag;
    if (htim->Instance == TIM3)
    {
        dbg_tim3_int_cnt++;
        if (dbg_tim3_int_cnt >= 1000)
        {
            timer_1ms++;
        }
        gzll_timer_isr_function();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t flag;
    if (GPIO_Pin == GPIO_PIN_1)
    {
        dbg_exit1_int_cnt++;
        //flag = gzll_interupts_save();
        do
        {
            gzll_radio_isr_function();
        } while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET);
        //gzll_interupts_restore(flag);
    }
}

