#ifndef __PPM_DECODER_H__
#define __PPM_DECODER_H__
/*
 * PPM decoder tuning parameters
 */
#define PPM_MIN_PULSE_WIDTH	    (200)		/**< minimum width of a valid first pulse */
#define PPM_MAX_PULSE_WIDTH	    (600)		/**< maximum width of a valid first pulse */
#define PPM_MIN_CHANNEL_VALUE   (800)		/**< shortest valid channel signal */
#define PPM_MAX_CHANNEL_VALUE   (2200)		/**< longest valid channel signal */
#define PPM_MIN_START           (2300)		/**< shortest valid start gap (only 2nd part of pulse) */

/* decoded PPM buffer */
#define PPM_MIN_CHANNELS	(5)
#define PPM_MAX_CHANNELS	(20)

/** Number of same-sized frames required to 'lock' */
#define PPM_CHANNEL_LOCK    (4)     /**< should be less than the input timeout */

#define PPM_DECODER_OVERFLOW_ERROR     (1)

typedef enum 
{
	UNSYNCH = 0,
	ARM,
	ACTIVE,
	INACTIVE
} PPM_DECODER_PHASE;

/** PPM decoder state machine */
typedef struct 
{
    uint16_t	last_edge;	/**< last capture time */
    uint16_t	last_mark;	/**< last significant edge */
    uint16_t	frame_start;	/**< the frame width */
    uint16_t	next_channel;	/**< next channel index */
    PPM_DECODER_PHASE phase;
} PPM_DECODER;

void ppm_decode(uint16_t count, uint16_t status);
void ppm_show_channels(void);

#endif
