#ifndef __PCM_DECODER_H__
#define __PCM_DECODER_H__

#define  PCM_MIN_START             (6000)
#define  PCM_FRAME_OVERFLOW_ERR    (1)
#define  PCM_OK                    (0)
#define  PCM_FAILED                (1)
#define  PCM_PPM_MAX               (12)
#define  PCM_PPM_MAX_CHAN_NUM      (16)

/* working mode */
#define  PCM_MODE_BINDING          (1)
#define  PCM_MODE_NORMAL           (0)

typedef enum 
{
	PCM_UNSYNCH = 0,   /* 0 */
	PCM_SYNC,          /* 1 */
	PCM_SYNC_0,        /* 2 */
	PCM_SYNC_1,        /* 3 */
	PCM_SYNC_2,        /* 4 */
	PCM_SYNC_3,        /* 5 */
	PCM_HEAD,          /* 6 */
	PCM_RX_NUM,        /* 7 */
	PCM_FLAG1,         /* 8 */
	PCM_FLAG2,         /* 9 */
	PCM_PPM,           /* 10 */
	PCM_RESERVE,       /* 11 */
	PCM_CRC_L,         /* 12 */
	PCM_CRC_H,         /* 13 */
	PCM_TAIL           /* 14 */
} PCM_DECODER_PHASE;

typedef struct
{
    uint8_t     rx_num;
    uint8_t     flag1;
    uint8_t     flag2;
    uint8_t     ppm_value[PCM_PPM_MAX];
    uint8_t     reserve;
} PCM_FRAME;

/** PCM decoder state machine */
typedef struct 
{
    uint32_t    frame_success;
    uint32_t    frame_tail_success;
    uint32_t    frame_crc_error;
    uint32_t    frame_sync_error;
    uint32_t    frame_head_error;
    uint32_t    frame_tail_error;
    uint16_t	last_edge;	/**< last capture time */
    uint16_t	bytes;
    uint16_t    crc;
    uint16_t    crc_calc;
    uint8_t     crc_l;
    uint8_t     crc_h;
    uint16_t    channel[PCM_PPM_MAX_CHAN_NUM];
    
    PCM_DECODER_PHASE phase;
    PCM_FRAME frame;
} PCM_DECODER;

void pcm_decode(uint16_t count, uint8_t status);
void pcm_decoder_init(void);
uint32_t pcm_mode_get(void);
uint8_t pcm_rxnum_get(void);
void pcm_ppm_channel_get(uint8_t *addr, uint8_t len);

void pcm_dbg_decode_step(void);
void pcm_decoder_start(void);
void pcm_decoder_stop(void);
void pcm_decoder_reset(void);


void pcm_frame_show(void);
void pcm_channel_show(void);
#endif