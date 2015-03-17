

#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "hal_nrf.h"
#include "radio.h"
#include "ppm_decoder.h"
#include <gzll.h>
#include <gzp.h>

extern uint16_t ppm_buffer[PPM_MAX_CHANNELS];

/** The payload sent over the radio. Also contains the recieved data. 
 * Should be read with radio_get_pload_byte(). */
static uint8_t pload[RF_PAYLOAD_LENGTH];
static uint8_t ack_pload[RF_PAYLOAD_LENGTH];
/** The current status of the radio. Should be set with radio_set_status(), 
 * and read with radio_get_status().
 */

//static uint8_t content[RF_PAYLOAD_LENGTH + 1] = "Hello, world!1234567890qwertyuiop";

osSemaphoreId radio_sema;
osThreadId radioTaskHandle;

uint8_t radio_get_pload_byte (uint8_t byte_index)
{
  return pload[byte_index];
}

void radio_get_pload (uint8_t *pload_out)
{
    memcpy(pload_out, pload, RF_PAYLOAD_LENGTH);
}

void radio_active(void)
{
    (void)osSemaphoreReleaseFromISR(radio_sema);
}


#ifndef GZLL_HOST_ONLY
int32_t radio_device_init(void)
{
    osSemaphoreDef(RADIO_SEM);
    radio_sema = osSemaphoreEmptyCreate(osSemaphore(RADIO_SEM));
    if (NULL == radio_sema)
    {
        printf("[%s, L%d] create semaphore failed ret 0x%x.\r\n", 
            __FILE__, __LINE__, (unsigned int)radio_sema);
        return -1;
    }

    osThreadDef(radioTask, radio_device_task, osPriorityNormal, 0, 2048);
    radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);
    if (NULL == radioTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n", 
            __FILE__, __LINE__);
        return -1;        
    }
    
    return 0;
}

bool pairing_ok = false;
void radio_device_task(void const *argument)
{
    bool tx_success = false;

    
    gzll_init();
    gzp_init();
    
    for(;;)
    {
        /* waitting for data update notify */
        (void)osSemaphoreWait(radio_sema, osWaitForever);

        #if 0
        if (gzll_get_state() == GZLL_IDLE)
        {
            memcpy(pload, ppm_buffer, RF_PAYLOAD_LENGTH);
            tx_success = gzll_tx_data(pload, GZLL_MAX_FW_PAYLOAD_LENGTH, 0);
            if (!tx_success)
            {
                printf("call gzll_tx_data failed!\r\n");
            }

            if (gzll_rx_fifo_read(ack_pload, NULL, NULL))
            {
                printf("Get ack pload 0x%x%x%x%x%x\r\n", ack_pload[0], ack_pload[1],
                    ack_pload[2], ack_pload[3], ack_pload[4]);                
            }
        }
        #endif
        
        #if 1
        if (!pairing_ok)
        {
            pairing_ok = gzp_address_req_send();            
        }
        else
        {
            if (gzll_get_state() == GZLL_IDLE)
            {
                memcpy(pload, ppm_buffer, RF_PAYLOAD_LENGTH);
                if (gzll_tx_data(pload, GZLL_MAX_FW_PAYLOAD_LENGTH, 2))
                {
                    
                    if (gzll_rx_fifo_read(ack_pload, NULL, NULL))
                    {
                        printf("Get ack pload 0x%x%x%x%x%x\r\n", ack_pload[0], ack_pload[1],
                            ack_pload[2], ack_pload[3], ack_pload[4]);
                    }
                }
                else
                {
                    printf("call gzll_tx_data failed!\r\n");
                }
            }
        }
        #endif
    }
}
#endif

#ifndef GZLL_DEVICE_ONLY
int32_t radio_host_init(void)
{
    osThreadDef(radioTask, radio_host_task, osPriorityNormal, 0, 2048);
    radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);
    if (NULL == radioTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n", 
            __FILE__, __LINE__);
        return -1;        
    }

    return 0;
}


void radio_host_task(void const * argument)
{
    argument = argument;

    gzll_init();
    gzp_init();
    gzp_pairing_enable(true);

    gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 2));

    gzll_rx_start();

    for (;;)
    {
        #if 0
        if (gzll_rx_fifo_read(pload, NULL, NULL))
        {
            printf("pload:%d\r\n", *(uint16_t*)pload);

            ack_pload[0] = 0x55;
            ack_pload[1] = 0x5a;
            ack_pload[2] = 0x55;
            ack_pload[3] = 0x5a;
            ack_pload[4] = 0x55;

            gzll_ack_payload_write(ack_pload, 5, 0);
        }
        #endif
        
        gzp_host_execute();
        
        if (gzll_get_rx_data_ready_pipe_number() == 2)
        {
            if (gzll_rx_fifo_read(pload, NULL, NULL))
            {
                printf("pload:%d\r\n", *(uint16_t*)pload);
                
                ack_pload[0] = 0x55;
                ack_pload[1] = 0x5a;
                ack_pload[2] = 0x55;
                ack_pload[3] = 0x5a;
                ack_pload[4] = 0x55;

                gzll_ack_payload_write(ack_pload, 5, 2);
            }
        }
    }
}
#endif


