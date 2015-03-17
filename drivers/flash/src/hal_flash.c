#include "hal_flash.h"

uint8_t hal_flash_byte_read(uint16_t a)
{
    return 0;
}

void hal_flash_bytes_read(uint16_t a, uint8_t *p, uint16_t n)
{  
    return;
}

void hal_flash_bytes_write(uint16_t a, const uint8_t *p, uint16_t n)
{

}

void hal_flash_byte_write(uint16_t a, uint8_t b)
{

}

void hal_flash_page_erase(uint8_t pn)
{

}

void gzp_host_chip_id_read(uint8_t *dst, uint8_t n)
{
    dst[0] = 0x33;
    dst[1] = 0x44;
    dst[2] = 0x55;
    dst[3] = 0x01;

    return;
}

void delay_us(uint16_t us)
{
    uint32_t i;
    do
    {   i = 72;
        do
        {
        } while(--i);
    } while (--us);
}
