
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include <stdio.h>

#define SDAH(i2c)  HAL_GPIO_WritePin(i2c->wire.sda.port, i2c->wire.sda.port_num, GPIO_PIN_SET)
#define SDAL(i2c)  HAL_GPIO_WritePin(i2c->wire.sda.port, i2c->wire.sda.port_num, GPIO_PIN_RESET)

#define SDA(i2c)       HAL_GPIO_ReadPin(i2c->wire.sda.port, i2c->wire.sda.port_num)
#define SDA_IN(i2c)    i2c_pin_dir_set(i2c->wire.sda.port, i2c->wire.sda.port_num, I2C_PIN_DIR_IN)
#define SDA_OUT(i2c)   i2c_pin_dir_set(i2c->wire.sda.port, i2c->wire.sda.port_num, I2C_PIN_DIR_OUT)

#define SCLH(i2c)  HAL_GPIO_WritePin(i2c->wire.scl.port, i2c->wire.scl.port_num, GPIO_PIN_SET)
#define SCLL(i2c)  HAL_GPIO_WritePin(i2c->wire.scl.port, i2c->wire.scl.port_num, GPIO_PIN_RESET)

#define STM32_DELAY_US_MULT         (12)

void i2c_delay_us(unsigned int us);
void I2C_delay(void);
void I2C_Start(I2C_INSTANCE_STRU *i2c);
void I2C_Stop(I2C_INSTANCE_STRU *i2c);
void I2C_Ack(I2C_INSTANCE_STRU *i2c);
void I2C_NoAck(I2C_INSTANCE_STRU *i2c);
void I2C_SendByte(I2C_INSTANCE_STRU *i2c, uint8_t SendByte);
uint8_t I2C_ReceiveByte(I2C_INSTANCE_STRU *i2c);
void I2C_WaitAck(I2C_INSTANCE_STRU *i2c);
void i2c_pin_dir_set(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t dir);

int32_t i2c_init(I2C_INSTANCE_STRU *i2c, I2C_WIRE_STRU *i2c_wire)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (NULL == i2c || NULL == i2c_wire)
    {
        return I2C_RET_PARAM;
    }

    i2c->mutex = osMutexCreate(NULL);
    if (NULL == i2c->mutex)
    {
        printf("[%s, L%d]osMutexCreate failed!\r\n", __FILE__, __LINE__);
        return I2c_RET_NO_RESROUCE;
    }

    i2c->wire.sda.port     = i2c_wire->sda.port;
    i2c->wire.sda.port_num = i2c_wire->sda.port_num;
    i2c->wire.scl.port     = i2c_wire->scl.port;
    i2c->wire.scl.port_num = i2c_wire->scl.port_num;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = i2c_wire->scl.port_num;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(i2c_wire->scl.port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(i2c_wire->scl.port, i2c_wire->scl.port_num, GPIO_PIN_SET);
    
    GPIO_InitStruct.Pin = i2c_wire->sda.port_num;
    HAL_GPIO_Init(i2c_wire->sda.port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(i2c_wire->sda.port, i2c_wire->sda.port_num, GPIO_PIN_SET);
    
    return I2C_RET_OK;
}

void i2c_delay_us(unsigned int us)
{
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r"(us)
                 : "r0");
}

void I2C_delay(void)
{
    i2c_delay_us(1);
}

void I2C_Start(I2C_INSTANCE_STRU *i2c)
{
    SDA_OUT(i2c);
    SDAH(i2c);
    SCLH(i2c);
    i2c_delay_us(4);
    SDAL(i2c);
    i2c_delay_us(4);
    SCLL(i2c);
    I2C_delay();
}

void I2C_Stop(I2C_INSTANCE_STRU *i2c)
{
    SDA_OUT(i2c);
    SCLL(i2c);
    SDAL(i2c);
    i2c_delay_us(4);
    SCLH(i2c);
    SDAH(i2c);
    i2c_delay_us(4);
}

void I2C_Ack(I2C_INSTANCE_STRU *i2c)
{
    SCLL(i2c);
    SDA_OUT(i2c);
    SDAL(i2c);
    
    i2c_delay_us(2);
    SCLH(i2c);
    i2c_delay_us(2);
    SCLL(i2c);
}


void I2C_NoAck(I2C_INSTANCE_STRU *i2c)
{
    SCLL(i2c);
    SDA_OUT(i2c);
    SDAH(i2c);
    
    i2c_delay_us(2);
    SCLH(i2c);
    i2c_delay_us(2);
    SCLL(i2c);
}

void I2C_SendByte(I2C_INSTANCE_STRU *i2c, uint8_t SendByte)
{
    unsigned char i = 8;

    SDA_OUT(i2c);
    while (i--)
    {
        SCLL(i2c);
        i2c_delay_us(2);

        if (SendByte & 0x80)
        {
            SDAH(i2c);
        }

        if (!(SendByte & 0x80))
        {
            SDAL(i2c);
        }

        SendByte <<= 1;
        i2c_delay_us(2);
        SCLH(i2c);
        i2c_delay_us(2);
    }

    SCLL(i2c);
}

uint8_t I2C_ReceiveByte(I2C_INSTANCE_STRU *i2c)
{
    uint8_t i = 8;
    uint8_t ReceiveByte = 0;
    uint8_t t;
    uint8_t data;

    SDA_IN(i2c);

    while (i--)
    {
        ReceiveByte <<= 1;
        SCLL(i2c);
        i2c_delay_us(2);
        SCLH(i2c);

        data = 0;

        for (t = 0; t < 8; t++)
        {
            data += SDA(i2c);
        }

        if (data >= 4)
        {
            ReceiveByte |= 0x01;
        }

    }

    SCLL(i2c);
    return ReceiveByte;
}

void I2C_WaitAck(I2C_INSTANCE_STRU *i2c)
{
    uint8_t cnt = 0;
    
    SDA_IN(i2c);
    SDAH(i2c);
    I2C_delay();
    SCLH(i2c);
    I2C_delay();

    while (SDA(i2c) == 1)
    {
        cnt++;
        if (cnt > 250)
        {
            i2c->error_num = 1;
            i2c->error_cnt++;
            I2C_Stop(i2c);
            return;
        }
    }

    SCLL(i2c);

    return;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
/**
 * @brief   Transmits data via the I2C bus as master.
 * @details Number of receiving bytes must be 0 or more than 1 on STM32F1x.
 *          This is hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
int32_t i2c_master_transmit
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t addr,
    uint8_t *txbuf, 
    size_t txbytes,
    uint8_t *rxbuf, 
    size_t rxbytes
) 
{
    uint16_t i;
    
    if (txbytes == 0)
    {
        return I2C_RET_OK;
    }

    osMutexWait(i2c->mutex, osWaitForever);

    I2C_Start(i2c);
    I2C_SendByte(i2c, addr << 1);
    I2C_WaitAck(i2c);

    for (i = 0; i < txbytes; i++)
    {
        I2C_SendByte(i2c, txbuf[i]);
        I2C_WaitAck(i2c);
    }

    if (rxbytes > 0)
    {
        I2C_Start(i2c);
        I2C_SendByte(i2c, (addr << 1) | 0x1);
        I2C_WaitAck(i2c);
    }

    for (i = 0; i < rxbytes; i++)
    {
        rxbuf[i] = I2C_ReceiveByte(i2c);
        if (i < rxbytes - 1)
        {
            I2C_Ack(i2c);
        }
        else
        {
            I2C_NoAck(i2c);
        }
    }

    I2C_Stop(i2c);
    osMutexRelease(i2c->mutex);

    return I2C_RET_OK;
    
}

uint32_t i2c_get_errors(I2C_INSTANCE_STRU *i2c)
{
    uint32_t ret;

    //osMutexWait(i2c->mutex, osWaitForever);
    ret = i2c->error_cnt;
    //osMutexRelease(i2c->mutex);

    return ret;
}

void i2c_pin_dir_set(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t dir)
{
    uint16_t position;
    uint16_t ioposition = 0;
    uint16_t iocurrent;
    __IO uint32_t *configregister; /* Store the address of CRL or CRH register 
                                      based on pin number */
    uint32_t registeroffset = 0;   /* offset used during computation of CNF and 
                                      MODE bits placement inside CRL or CRH 
                                      register */
  
    /* Configure the port pins */
    for (position = 0; position < 16; position++)
    {
        /* Get the IO position */
        ioposition = ((uint32_t)0x01) << position;

        /* Get the current IO position */
        iocurrent = pin & ioposition;

        if (iocurrent == ioposition)
        {
            /* Check if the current bit belongs to first half or last half of the pin count number
            in order to address CRH or CRL register*/
            configregister = (iocurrent < GPIO_PIN_8) ? &GPIOx->CRL     : &GPIOx->CRH;
            registeroffset = (iocurrent < GPIO_PIN_8) ? (position << 2) : ((position - 8) << 2);

            *configregister &= 0xf << registeroffset; 
            
            if (dir == I2C_PIN_DIR_IN)
            {
                *configregister |= 8 << registeroffset;
            }
            else
            {
                *configregister |= 3 << registeroffset;
            }
        }
    }

    return;
}

