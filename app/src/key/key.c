#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "key.h"

void key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* key 0 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* key 1 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    
    return;
}

uint8_t key_scan(uint8_t mode)
{
	static uint8_t key_up = 1; //按键按松开标志
	
	if (mode)
	{
	    key_up=1;  //支持连按
	}
	
	if (key_up 
	&& ((KEY0 == GPIO_PIN_RESET) || (KEY1 == GPIO_PIN_RESET)))
	{
		osDelay(10);//去抖动 
		key_up = 0;
		
		if (KEY0 == GPIO_PIN_RESET)
		{
		    return KEY0_PRES;
		}
		else if (KEY1 == GPIO_PIN_RESET)
		{
		    return KEY1_PRES;
		}
	}
	else if ((KEY0 == GPIO_PIN_SET) && (KEY1 == GPIO_PIN_SET))
	{
	    key_up=1;
	}
	
	return 0;
}

