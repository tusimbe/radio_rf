#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "pcm_decoder.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim2;
PCM_DECODER pcm;

const uint16_t crc_table[] =
{
  0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
  0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
  0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
  0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
  0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
  0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
  0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
  0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
  0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
  0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
  0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
  0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
  0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
  0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
  0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
  0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
  0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
  0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
  0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
  0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
  0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
  0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
  0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
  0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
  0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
  0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
  0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
  0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
  0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
  0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
  0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
  0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
};

PCM_FRAME g_frame_tmp;
uint16_t debug_width[200];
uint16_t debug_width_idx;
uint32_t debug_decode_stop;
PCM_DECODER_PHASE phase_debug;

/* extern function */
extern void radio_active(void);


/* internal fucntion */
uint32_t pcm_get_part(uint16_t width);
uint16_t pcm_crc_get(uint16_t crc_init, uint8_t *data, uint16_t len);
uint32_t pcm_get_bit(uint16_t width, uint8_t *bit);
uint32_t pcm_get_head(uint32_t bit_len, uint8_t *byte);
uint32_t pcm_get_byte(uint32_t bit_len, uint8_t *byte);
void pcm_channel_get(uint8_t *ppm, uint16_t *buf);

void pcm_debug_record_width(uint16_t width)
{    
    if (debug_width_idx < 200)
    {
        debug_width[debug_width_idx] = width;
        debug_width_idx++;
    }
}

void pcm_debug_record_width_by_idx(uint16_t width, uint16_t idx)
{
    if (idx >= 100)
    {
        return;
    }
    debug_width[idx] = width;
}

void pcm_decoder_init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 36 - 1; /* 2MHZ */
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

uint32_t pcm_get_part(uint16_t width)
{                    
    if (width > 16 && width < 35)
    {
        return 0;
    }
    else if (width >= 35 && width <= 80)
    {
        return 1;
    }
    else
    {
        printf("part error width:%d.\r\n", width);
        return -1;
    }
}

uint16_t pcm_crc_get(uint16_t crc_init, uint8_t *data, uint16_t len)
{
    uint16_t crc = crc_init;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ (crc_table[((crc >> 8) ^ data[i]) & 0xff]);
    }

    return crc;
}

uint32_t pcm_get_bit(uint16_t width, uint8_t *bit)
{
    static uint8_t ones = 0;
    static uint8_t next_is_stuff = 0;
    uint8_t part;

    part = pcm_get_part(width);
    if (part)
    {
        ones++;
    }
    else
    {
        ones = 0;
    }

    if (next_is_stuff)
    {
        next_is_stuff = 0;
        return PCM_FAILED;
    }
    
    if (ones >= 5)
    {
        next_is_stuff = 1;
    }

    *bit = part;
    return PCM_OK;
}

uint32_t pcm_get_head(uint32_t bit_len, uint8_t *byte)
{
    static uint8_t bits = 7;
    static uint8_t tmp = 0;

    if (bits > 0)
    {
        tmp |= pcm_get_part(bit_len) << bits;
        bits--;
        return PCM_FAILED;
    }
    else
    {
        tmp |= pcm_get_part(bit_len);
        bits = 7;
        *byte = tmp;
        tmp = 0;
        return PCM_OK;
    }    
}

uint32_t pcm_get_byte(uint32_t bit_len, uint8_t *byte)
{
    static uint8_t bits = 7;
    static uint8_t tmp = 0;
    uint8_t bit;
    uint32_t ret;

    if (bits > 0)
    {
        ret = pcm_get_bit(bit_len, &bit);
        if (ret == PCM_OK)
        {
            tmp |= bit << bits;
            bits--;
        }
        
        return PCM_FAILED;
    }
    else
    {
        ret = pcm_get_bit(bit_len, &bit);
        if (ret != PCM_OK)
        {
            return PCM_FAILED;
        }

        tmp |= bit;
        bits = 7;
        *byte = tmp;
        tmp = 0;
        return PCM_OK;
    }
}

void pcm_phase_record(void)
{
    if (pcm.phase > phase_debug)
    {
        phase_debug = pcm.phase;
    }
}

void pcm_decode_process(uint16_t width, uint8_t status)
{
    uint8_t  byte;

    if (width >= PCM_MIN_START)
    {
        pcm.phase = PCM_SYNC;
    }
    
    pcm_phase_record();
    
    switch (pcm.phase)
    {
        case PCM_SYNC:
            pcm.phase = PCM_SYNC_0;
            break;
        case PCM_SYNC_0:
            if (0 == pcm_get_part(width + 20))
            {
                pcm.phase = PCM_SYNC_1;
            }
            else
            {
                pcm.frame_sync_error++;
                pcm.phase = PCM_UNSYNCH;
            }
            break;
        case PCM_SYNC_1:
            if (0 == pcm_get_part(width))
            {
                pcm.phase = PCM_SYNC_2;
            }
            else
            {
                pcm.frame_sync_error++;
                pcm.phase = PCM_UNSYNCH;
            }
            break;    
        case PCM_SYNC_2:
            if (0 == pcm_get_part(width))
            {
                pcm.phase = PCM_SYNC_3;
            }
            else
            {
                pcm.frame_sync_error++;
                pcm.phase = PCM_UNSYNCH;
            }
            break;    
        case PCM_SYNC_3:
            if (0 == pcm_get_part(width))
            {
                pcm.phase = PCM_HEAD;
            }
            else
            {
                pcm.frame_sync_error++;
                pcm.phase = PCM_UNSYNCH;
            }
            break;
        case PCM_HEAD:
            if (pcm_get_head(width, &byte) == PCM_OK)
            {
                if (byte == 0x7e)
                {
                    pcm.phase = PCM_RX_NUM;
                }
                else
                {
                    pcm.frame_head_error++;
                    pcm.phase = PCM_UNSYNCH;
                }
            }
            break;
        case PCM_RX_NUM:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                g_frame_tmp.rx_num = byte;
                pcm.phase = PCM_FLAG1;
            }
            break;
        case PCM_FLAG1:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                g_frame_tmp.flag1 = byte;
                pcm.phase = PCM_FLAG2;
            }
            break;
        case PCM_FLAG2:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                g_frame_tmp.flag2 = byte;
                pcm.phase = PCM_PPM;
            }
            break;
        case PCM_PPM:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                g_frame_tmp.ppm_value[pcm.bytes] = byte;
                pcm.bytes++;
            }

            if (pcm.bytes >= PCM_PPM_MAX)
            {
                pcm.bytes = 0;
                pcm.phase = PCM_RESERVE;
            }
            break;
        case PCM_RESERVE:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                g_frame_tmp.reserve = byte;
                pcm.phase = PCM_CRC_L;
            }
            break;
        case PCM_CRC_L:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                pcm.crc_h = byte;
                pcm.phase = PCM_CRC_H;
            }
            break;
        case PCM_CRC_H:
            if (pcm_get_byte(width, &byte) == PCM_OK)
            {
                pcm.crc_l = byte;
                pcm.crc = (pcm.crc_h << 8) | byte;
                pcm.phase = PCM_TAIL;
            }
            break;
        case PCM_TAIL:
            if (pcm_get_head(width, &byte) == PCM_OK)
            {
                /* crc check */
                if (0x7e == byte)
                {
                    pcm.frame_tail_success++;
                    pcm.phase = PCM_UNSYNCH;
                    TIM2->CCER ^= TIM_CCER_CC1P;

                    pcm.crc_calc = 
                        pcm_crc_get(0, &g_frame_tmp.rx_num, sizeof(PCM_FRAME));
                    if (pcm.crc_calc == pcm.crc)
                    {
                        memcpy(&pcm.frame, &g_frame_tmp, PCM_PPM_MAX);
                        pcm_channel_get(&g_frame_tmp.ppm_value, pcm.channel);
                        pcm.frame_success++;
                        radio_active();
                    }
                    else
                    {
                        pcm.frame_crc_error++;
                    }
                }
                else
                {
                    pcm.frame_tail_error++;
                }
            }
            break;
        default:
            pcm.phase = PCM_UNSYNCH;
            break;
    }        
}

void pcm_decode(uint16_t count, uint8_t status)
{
    uint16_t width;
    static uint32_t first_frame = 0;
    static uint16_t record = 0;
        
    if (debug_decode_stop)
    {
        return;
    }

    if (status & PCM_FRAME_OVERFLOW_ERR)
    {
        pcm.phase = PCM_UNSYNCH;
        return;
    }
    
    if (pcm.phase == PCM_UNSYNCH)
    {
        TIM2->CCER ^= TIM_CCER_CC1P;
    }

    width = count - pcm.last_edge;
    /* new frame arrive */
    if (width >= PCM_MIN_START)
    {
        if (first_frame == 0)
        {
            first_frame = 1;
        }
        else
        {
            pcm.phase = PCM_SYNC;
            record = 1;
        }
    }
    
    pcm.last_edge = count;

    pcm_decode_process(width, status);
    
    if (record)
    {
        pcm_debug_record_width(width);
    }
    return;
    
}

uint32_t pcm_mode_get(void)
{
    if (pcm.frame.flag1 & 0x01)
    {
        return PCM_MODE_BINDING;
    }
    else
    {
        return PCM_MODE_NORMAL;
    }
}

uint8_t pcm_rxnum_get(void)
{
    return pcm.frame.rx_num;
}

void pcm_ppm_channel_get(uint8_t *addr, uint8_t len)
{
    memcpy(addr, pcm.channel, len);
}

void pcm_decoder_reset(void)
{
    memset(&pcm, 0, sizeof(PCM_DECODER));
    memset(&g_frame_tmp, 0, sizeof(PCM_FRAME));
}

void pcm_decoder_stop(void)
{
    debug_decode_stop = 1;
}

void pcm_decoder_start(void)
{
    debug_decode_stop = 0;
}

void pcm_frame_show(void)
{
    uint32_t i;

    printf("PCM success frame:%d, PCM tail success frame:%d\r\n", 
        pcm.frame_success, pcm.frame_tail_success);
        
    printf("crc_error:%d, head_err:%d, sync_err:%d, tail_err:%d\r\n",
        pcm.frame_crc_error, pcm.frame_head_error, 
        pcm.frame_sync_error, pcm.frame_tail_error);

    printf("bytes:%d, crc: 0x%04x, crc calc: 0x%04x\r\n", pcm.bytes, pcm.crc, pcm.crc_calc);
        
    printf("PCM rx num: %d, flag1: 0x%x, flag2: 0x%x\r\n", 
        g_frame_tmp.rx_num, g_frame_tmp.flag1, g_frame_tmp.flag2);

    printf("PCM PPM:");
    for (i = 0; i < PCM_PPM_MAX; i++)
    {
        printf("%02x ", g_frame_tmp.ppm_value[i]);
    }

    printf("\r\n");
    
    printf("phase: %d, width idx:%d \r\n", phase_debug, debug_width_idx);
    for (i = 0; i < debug_width_idx; i++)
    {
        printf("%d ", debug_width[i]);
    }

    printf("\r\n");

    return;
}

void pcm_dbg_decode_step(void)
{
    static uint16_t frame_ptr = 0;
    uint32_t i;

    printf("get width: %d @pos %d\r\n", debug_width[frame_ptr], frame_ptr);
    pcm_decode_process(debug_width[frame_ptr], 0);
    frame_ptr++;

    if (frame_ptr >= debug_width_idx)
    {
        frame_ptr = 0;
    }

    printf("bytes:%d, crc_l:0x%02x, crc_h:0x%02x, crc:0x%04x\r\n", 
        pcm.bytes, pcm.crc_l, pcm.crc_h, pcm.crc);
        
    printf("PCM rx num: %d, flag1: 0x%x, flag2: 0x%x, phase:%d\r\n", 
        g_frame_tmp.rx_num, g_frame_tmp.flag1, g_frame_tmp.flag2, pcm.phase);

    printf("PCM PPM:");
    for (i = 0; i < PCM_PPM_MAX; i++)
    {
        printf("%02x ", g_frame_tmp.ppm_value[i]);
    }

    printf("\r\n");
}

void pcm_channel_get(uint8_t *ppm, uint16_t *buf)
{
    uint16_t channel;

    /* channel 1/9 */
    channel = ((ppm[1] & 0xf) << 8) | ppm[0];
    if (channel > 2047)
    {
        buf[8] = channel & 0x7ff;
    }
    else
    {
        buf[0] = channel;
    }

    
    /* channel 2/10 */
    channel = (ppm[2] << 4) | ((ppm[1] & 0xf0) >> 4);
    if (channel > 2047)
    {
        buf[9] = channel & 0x7ff;
    }
    else
    {
        buf[1] = channel;
    }

    /* channel 3/11 */
    channel = ((ppm[4] & 0xf) << 8) | ppm[3];
    if (channel > 2047)
    {
        buf[10] = channel & 0x7ff;
    }
    else
    {
        buf[2] = channel;
    }
    
    /* channel 4/12 */
    channel = (ppm[5] << 4) | ((ppm[4] & 0xf0) >> 4);
    if (channel > 2047)
    {
        buf[11] = channel & 0x7ff;
    }
    else
    {
        buf[3] = channel;
    }

    /* channel 5/13 */
    channel = ((ppm[7] & 0xf) << 8) | ppm[6];
    if (channel > 2047)
    {
        buf[12] = channel & 0x7ff;
    }
    else
    {
        buf[4] = channel;
    }
    
    /* channel 6/14 */
    channel = (ppm[8] << 4) | ((ppm[7] & 0xf0) >> 4);
    if (channel > 2047)
    {
        buf[13] = channel & 0x7ff;
    }
    else
    {
        buf[5] = channel;
    }

    /* channel 7/15 */
    channel = ((ppm[10] & 0xf) << 8) | ppm[9];
    if (channel > 2047)
    {
        buf[14] = channel & 0x7ff;
    }
    else
    {
        buf[6] = channel;
    }
    
    /* channel 8/16 */
    channel = (ppm[11] << 4) | ((ppm[10] & 0xf0) >> 4);
    if (channel > 2047)
    {
        buf[15] = channel & 0x7ff;
    }
    else
    {
        buf[7] = channel;
    }

    return;
}

void pcm_channel_show(void)
{
    uint16_t i;

    for (i = 0; i < PCM_PPM_MAX_CHAN_NUM; i++)
    {
        printf("chan %d %04d\r\n", i, pcm.channel[i]);
    }
}
