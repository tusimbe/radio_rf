
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ppm_decoder.h"

extern TIM_HandleTypeDef htim2;

uint16_t ppm_buffer[PPM_MAX_CHANNELS];
uint16_t ppm_frame_length = 0;
unsigned ppm_decoded_channels = 0;
uint64_t ppm_last_valid_decode = 0;

#define PPM_DEBUG 0

#if PPM_DEBUG
uint16_t ppm_edge_history[32];
unsigned ppm_edge_next;

/* PPM pulse history */
uint16_t ppm_pulse_history[32];
unsigned ppm_pulse_next;
#endif

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];

/* PPM decoder state machine */
PPM_DECODER ppm;

extern void radio_active(void);
uint64_t ppm_absolute_time(void);

void ppm_decode(uint16_t count, uint16_t status)
{
	uint16_t width;
	uint16_t interval;
	unsigned i;

	/* if we missed an edge, we have to give up */
	if (status & PPM_DECODER_OVERFLOW_ERROR)
		goto error;

	/* how long since the last edge? - this handles counter wrapping implicitely. */
	width = count - ppm.last_edge;

#if PPM_DEBUG
	ppm_edge_history[ppm_edge_next++] = width;

	if (ppm_edge_next >= 32)
		ppm_edge_next = 0;
#endif

	/*
	 * if this looks like a start pulse, then push the last set of values
	 * and reset the state machine
	 */
	if (width >= PPM_MIN_START) {

		/*
		 * If the number of channels changes unexpectedly, we don't want
		 * to just immediately jump on the new count as it may be a result
		 * of noise or dropped edges.  Instead, take a few frames to settle.
		 */
		if (ppm.next_channel != ppm_decoded_channels) {
			static unsigned new_channel_count;
			static unsigned new_channel_holdoff;

			if (new_channel_count != ppm.next_channel) {
				/* start the lock counter for the new channel count */
				new_channel_count = ppm.next_channel;
				new_channel_holdoff = PPM_CHANNEL_LOCK;

			} else if (new_channel_holdoff > 0) {
				/* this frame matched the last one, decrement the lock counter */
				new_channel_holdoff--;

			} else {
				/* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
				ppm_decoded_channels = new_channel_count;
				new_channel_count = 0;
			}

		} else {
			/* frame channel count matches expected, let's use it */
			if (ppm.next_channel > PPM_MIN_CHANNELS) {
				for (i = 0; i < ppm.next_channel; i++)
					ppm_buffer[i] = ppm_temp_buffer[i];

				ppm_last_valid_decode = ppm_absolute_time();
				radio_active();
			}
		}

		/* reset for the next frame */
		ppm.next_channel = 0;

		/* next edge is the reference for the first channel */
		ppm.phase = ARM;

		ppm.last_edge = count;
		return;
	}

	switch (ppm.phase) {
	case UNSYNCH:
		/* we are waiting for a start pulse - nothing useful to do here */
		break;

	case ARM:

		/* we expect a pulse giving us the first mark */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH)
			goto error;		/* pulse was too short or too long */

		/* record the mark timing, expect an inactive edge */
		ppm.last_mark = ppm.last_edge;

		/* frame length is everything including the start gap */
		ppm_frame_length = (uint16_t)(ppm.last_edge - ppm.frame_start);
		ppm.frame_start = ppm.last_edge;
		ppm.phase = ACTIVE;
		break;

	case INACTIVE:

		/* we expect a short pulse */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH)
			goto error;		/* pulse was too short or too long */

		/* this edge is not interesting, but now we are ready for the next mark */
		ppm.phase = ACTIVE;
		break;

	case ACTIVE:
		/* determine the interval from the last mark */
		interval = count - ppm.last_mark;
		ppm.last_mark = count;

#if PPM_DEBUG
		ppm_pulse_history[ppm_pulse_next++] = interval;

		if (ppm_pulse_next >= 32)
			ppm_pulse_next = 0;
#endif

		/* if the mark-mark timing is out of bounds, abandon the frame */
		if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE))
			goto error;

		/* if we have room to store the value, do so */
		if (ppm.next_channel < PPM_MAX_CHANNELS)
			ppm_temp_buffer[ppm.next_channel++] = interval;

		ppm.phase = INACTIVE;
		break;

	}

	ppm.last_edge = count;
	return;

	/* the state machine is corrupted; reset it */

error:
	/* we don't like the state of the decoder, reset it and try again */
	ppm.phase = UNSYNCH;
	ppm_decoded_channels = 0;

    return;
}

uint64_t ppm_absolute_time(void)
{
	uint64_t	abstime;
	uint32_t	count;
	//irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	static volatile uint64_t base_time;
	static volatile uint32_t last_count;

	/* prevent re-entry */
	//flags = irqsave();

	/* get the current counter value */
	count = htim2.Instance->CNT;

	/*
	 * Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count < last_count)
		base_time += 65535;

	/* save the count for next time */
	last_count = count;

	/* compute the current time */
	abstime = base_time + count;

	//irqrestore(flags);

	return abstime;
}

void ppm_show_channels(void)
{
    int i;

    for (i = 0; i < PPM_MAX_CHANNELS; i++)
    {
        printf("channel %02d PWM %04d\r\n", i, ppm_buffer[i]);
    }

    return;
}
