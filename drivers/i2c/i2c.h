#ifndef __I2C_H__
#define __I2C_H__

#define  I2C_RET_OK             (0)
#define  I2C_RET_PARAM          (-1)
#define  I2c_RET_NO_RESROUCE    (-2)

#define  I2C_PIN_DIR_IN         (0)
#define  I2C_PIN_DIR_OUT        (1)

typedef struct i2c_io_s
{
    GPIO_TypeDef *port;
    uint16_t port_num;
} I2C_IO_STRU;

typedef struct i2c_wire_s
{
    I2C_IO_STRU scl;
    I2C_IO_STRU sda;
} I2C_WIRE_STRU;

typedef struct i2c_instance_s 
{
    osMutexId mutex;
    I2C_WIRE_STRU wire;
    uint32_t error_num;
    uint32_t error_cnt;
} I2C_INSTANCE_STRU;

int32_t i2c_master_transmit
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t addr,
    uint8_t *txbuf, 
    size_t txbytes,
    uint8_t *rxbuf, 
    size_t rxbytes
);

int32_t i2c_master_transmit_af
(
    uint8_t i2c,
    uint8_t addr,
    const uint8_t *txbuf, 
    size_t txbytes,
    uint8_t *rxbuf, 
    size_t rxbytes
);

void MX_I2C2_Init(void);
uint32_t i2c_get_errors(I2C_INSTANCE_STRU *i2c);
int32_t i2c_init(I2C_INSTANCE_STRU *i2c, I2C_WIRE_STRU *i2c_wire);
#endif
