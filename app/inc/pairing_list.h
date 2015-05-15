#ifndef __PAIRING_LIST_H__
#define __PAIRING_LIST_H__

#define  PAIRING_LIST_ADDR_LEN       (4)
#define  PAIRING_LIST_MAX_ADDR_NUM   (64)

int32_t pairing_list_init(void);
int32_t pairing_list_addr_read(uint8_t rx_num, uint8_t *addr);
int32_t pairing_list_addr_write(uint8_t rx_num, uint8_t *addr);

void pairing_list_entry_show(uint8_t idx);
#endif