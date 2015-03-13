

#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "hal_nrf.h"
#include "radio.h"
#include "ppm_decoder.h"

extern uint16_t ppm_buffer[PPM_MAX_CHANNELS];

static uint8_t address[HAL_NRF_AW_5BYTES] = {0x34,0x43,0x10,0x10,0x01};

/** The payload sent over the radio. Also contains the recieved data. 
 * Should be read with radio_get_pload_byte(). */
static uint8_t pload[RF_PAYLOAD_LENGTH];
/** The current status of the radio. Should be set with radio_set_status(), 
 * and read with radio_get_status().
 */
static radio_status_t status;

//static uint8_t content[RF_PAYLOAD_LENGTH + 1] = "Hello, world!1234567890qwertyuiop";

osSemaphoreId radio_sema;
static hal_nrf_operation_mode_t g_op_mode;

void radio_send_packet(uint8_t *packet, uint8_t length)
{
    CE_LOW();
    hal_nrf_write_tx_pload(packet, length);      // load message into radio
    CE_PULSE();                                   // send packet

    radio_set_status (RF_BUSY);                 // trans. in progress; RF_BUSY
}

radio_status_t radio_get_status (void)
{
  return status;
}

uint8_t radio_get_pload_byte (uint8_t byte_index)
{
  return pload[byte_index];
}

void radio_get_pload (uint8_t *pload_out)
{
    memcpy(pload_out, pload, RF_PAYLOAD_LENGTH);
}


void radio_set_status (radio_status_t new_status)
{
    printf("radio_set_status %d\r\n", new_status);
    status = new_status;
}

void radio_esb_init (uint8_t *address, hal_nrf_operation_mode_t operational_mode)
{
  hal_nrf_close_pipe(HAL_NRF_ALL);               // First close all radio pipes
                                                 // Pipe 0 and 1 open by default
  hal_nrf_open_pipe(HAL_NRF_PIPE0, true);        // Then open pipe0, w/autoack
                                                 // Changed from sb/radio_sb.c

  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);       // Operates in 16bits CRC mode
  hal_nrf_set_auto_retr(RF_RETRANSMITS, RF_RETRANS_DELAY);                 
                                                 // Enables auto retransmit.
                                                 // 3 retrans with 250ms delay
                                                 // Changed from sb/radio_sb.c

  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);  // 5 bytes address width
  hal_nrf_set_address(HAL_NRF_TX, address);      // Set device's addresses
  hal_nrf_set_address(HAL_NRF_PIPE0, address);   // Sets recieving address on 
                                                 // pipe0
  
  if(operational_mode == HAL_NRF_PTX)            // Mode depentant settings
  {
    hal_nrf_set_operation_mode(HAL_NRF_PTX);     // Enter TX mode
  }
  else
  {
    hal_nrf_set_operation_mode(HAL_NRF_PRX);     // Enter RX mode
    hal_nrf_set_rx_pload_width((uint8_t)HAL_NRF_PIPE0, RF_PAYLOAD_LENGTH);
                                                 // Pipe0 expect 
                                                 // PAYLOAD_LENGTH byte payload
                                                 // PAYLOAD_LENGTH in radio.h
  }

  hal_nrf_set_rf_channel(RF_CHANNEL);            // Operating on static channel 
                                                 // Defined in radio.h.
                                                 // Frequenzy = 
                                                 //        2400 + RF_CHANNEL
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);        // Power up device

  osDelay(20);
  
  radio_set_status (RF_IDLE);                     // Radio now ready
}  

void radio_irq(void) 
{
    uint8_t status;

    status = hal_nrf_get_clear_irq_flags();
    switch (status)
    {
        case (1<<HAL_NRF_MAX_RT):                 // Max retries reached
            hal_nrf_flush_tx();                     // flush tx fifo, avoid fifo jam
            radio_set_status (RF_MAX_RT);
            break;

        case (1<<HAL_NRF_TX_DS):                  // Packet sent
            radio_set_status (RF_TX_DS);
            break;

        case (1<<HAL_NRF_RX_DR):                  // Packet received
            while (!hal_nrf_rx_fifo_empty ())
            {
                hal_nrf_read_rx_pload(pload);
            }
            radio_set_status (RF_RX_DR);
            break;

        case ((1<<HAL_NRF_RX_DR)|(1<<HAL_NRF_TX_DS)): // Ack payload recieved
            while (!hal_nrf_rx_fifo_empty ())
            {
                hal_nrf_read_rx_pload(pload);
            }
            radio_set_status (RF_TX_AP);
            break;

        default:
        #if 0
            printf("1.radio irq check status = 0x%x\r\n", status);
            status = hal_nrf_read_reg(EN_AA);
            printf("2.EN_AA = 0x%x\r\n", status);
            status = hal_nrf_read_reg(EN_RXADDR);
            printf("3.EN_RXADDR = 0x%x\r\n", status);

            status = hal_nrf_read_reg(SETUP_RETR);
            printf("4.SETUP_RETR = 0x%x\r\n", status);
            status = hal_nrf_read_reg(RF_CH);
            printf("5.RF_CH = 0x%x\r\n", status);
            status = hal_nrf_read_reg(RF_SETUP);
            printf("6.RF_SETUP = 0x%x\r\n", status);
            status = hal_nrf_read_reg(CONFIG);
            printf("7.CONFIG = 0x%x\r\n", status);
        #endif
            break;    
    }
}

int32_t radio_init(hal_nrf_operation_mode_t op_mode)
{
    osSemaphoreDef(RADIO_SEM);
    radio_sema = osSemaphoreEmptyCreate(osSemaphore(RADIO_SEM));
    if (NULL == radio_sema)
    {
        printf("[%s, L%d] create semaphore failed ret 0x%x.\r\n", 
            __FILE__, __LINE__, (unsigned int)radio_sema);
        return -1;
    }
    
    g_op_mode = op_mode;
    return 0;
}

void radio_active(void)
{
    (void)osSemaphoreReleaseFromISR(radio_sema);
}

void StartRadioTask(void const * argument)
{
    uint32_t time;
    uint16_t *pChannel;
    int32_t i;
    
    argument = argument;

    radio_esb_init(address, g_op_mode);

    for (;;)
    {
        if (HAL_NRF_PTX == g_op_mode)
        {
            /* waitting for data update notify */
            osSemaphoreWait(radio_sema, osWaitForever);
        }

        time = 0;
        do {
            radio_irq ();
            time++;
            if (time > 100)
            {
                break;
            }
        } while((radio_get_status()) == RF_BUSY);

        if (HAL_NRF_PRX == g_op_mode)
        {
        #if 1
            pChannel = (uint16_t*)pload;
            for (i = 0; i < 16; i++)
            {
                printf("channel %02d PWM %04d\r\n", (int)i, *pChannel);
                pChannel++;
            }
        #endif
        }
        else
        {
            memcpy(pload, ppm_buffer, RF_PAYLOAD_LENGTH);
            radio_send_packet(pload, RF_PAYLOAD_LENGTH);
        }
    }
}

