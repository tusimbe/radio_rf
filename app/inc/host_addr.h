#ifndef __HOST_ADDR_H__
#define __HOST_ADDR_H__

int32_t host_addr_init(void);
void host_chip_id_write(uint8_t *addr, uint8_t len);
void host_chip_id_read(uint8_t *dst, uint8_t len);



#endif
