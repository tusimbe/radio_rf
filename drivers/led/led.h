#ifndef __LED_H__
#define __LED_H__

#define  LED_0     (0)
#define  LED_1     (1)

void led_blink
(
    uint8_t led, 
    uint8_t times, 
    uint32_t blink_interval
);

int32_t led_init(void);



#endif

