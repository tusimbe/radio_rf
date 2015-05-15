#ifndef __KEY_H__
#define __KEY_H__

#define KEY0  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)
#define KEY1  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)


#define KEY0_PRES       (1)		 
#define KEY1_PRES       (2)		

void key_init(void);
uint8_t key_scan(uint8_t mode);
#endif