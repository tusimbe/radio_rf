#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ppm_encoder.h"

int32_t ppm_encoder
(
    PPM_ENCODER *conf, 
    uint16_t *ppm_pulse_seqence, 
    uint16_t *ppm_last_low_width
)
{
    uint32_t i;
    uint32_t rest_frame_length = PPM_FRAME_LENGTH_DEFAULT;
    uint16_t *p_pulse_seq = ppm_pulse_seqence;
    uint16_t pulse_width_one_chan;
    

    for (i = 0; i < conf->valid_chan_num; i++)
    {
        if (conf->channels[i] == 0)
        {
            pulse_width_one_chan = PPM_CENTER;
        }
        else
        {
            pulse_width_one_chan = conf->channels[i];
        }
        rest_frame_length -= pulse_width_one_chan;
        *p_pulse_seq = pulse_width_one_chan;
        p_pulse_seq++;
    }

    if (rest_frame_length > 65535)
    {
        rest_frame_length = 65535;
    }

    if (rest_frame_length < 4500)
    {
        rest_frame_length = 4500;
    }

    *p_pulse_seq = rest_frame_length;
    *(p_pulse_seq + 1) = 0;

    *ppm_last_low_width = rest_frame_length;

    return 0;
    
}

