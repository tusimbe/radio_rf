#ifndef __PPM_ENCODER_H__
#define __PPM_ENCODER_H__

#define PPM_MAX_CHANNELS              (20)
#define PPM_RANGE                     (512)
#define PPM_CENTER                    (1500)
#define PPM_FRAME_LENGTH_DEFAULT      (22500)           /* 22.5 ms */
#define PPM_STOP_PULSE_WIDTH          (300)
  
typedef struct ppm_encoder_s
{
    uint16_t channels[PPM_MAX_CHANNELS];
    uint32_t valid_chan_num;
    uint16_t stop_pulse_width;    /* us */
} PPM_ENCODER;

int32_t ppm_encoder
(
    PPM_ENCODER *conf, 
    uint16_t *ppm_pulse_seqence, 
    uint16_t *ppm_last_low_width
);

#endif