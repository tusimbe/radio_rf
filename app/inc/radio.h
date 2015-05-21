#ifndef  __RADIO_H__
#define  __RADIO_H__
#include "stm32f1xx_hal.h"
#include "hal_nrf.h"

#define  RF_PAYLOAD_LENGTH   32

uint8_t radio_get_pload_byte (uint8_t byte_index);
void radio_get_pload (uint8_t *pload_out);
int32_t radio_device_init(void);
int32_t radio_host_init(void);
void radio_active(void);
void radio_device_task(void const *argument);
void radio_host_task(void const *argument);

void radio_pairing_status_set(bool status);


#endif
