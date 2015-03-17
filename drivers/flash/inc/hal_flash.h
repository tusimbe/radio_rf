#ifndef __HAL_FLASH_H__
#define __HAL_FLASH_H__
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

void delay_us(uint16_t us);
uint8_t hal_flash_byte_read(uint16_t a);
void hal_flash_bytes_read(uint16_t a, uint8_t *p, uint16_t n);
void hal_flash_bytes_write(uint16_t a, const uint8_t *p, uint16_t n);
void hal_flash_byte_write(uint16_t a, uint8_t b);
void hal_flash_page_erase(uint8_t pn);

#endif
