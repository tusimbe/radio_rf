/* Copyright (c) 2007 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2132 $
 */ 

/** @file
 * Header file defining the hardware depenent interface of the C8051F320
 *
 *
 */

#ifndef HAL_NRF_HW_H__
#define HAL_NRF_HW_H__

/** Macro that set radio's CSN line LOW.
 *
 */
#define CSN_LOW() \
    do            \
    {                \
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); \
    } while(0)

/** Macro that set radio's CSN line HIGH.
 *
 */
#define CSN_HIGH() \
    do             \
    {              \
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);  \
    } while(0)

/** Macro that set radio's CE line LOW.
 *
 */
#define CE_LOW() \
    do           \
    {            \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); \
    } while(0)

/** Macro that set radio's CE line HIGH.
 *
 */
#define CE_HIGH() \
    do            \
    {             \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); \
    } while(0)

#define IRQ()  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)

/** Macro for writing the radio SPI data register.
 *
 */
#define HAL_NRF_HW_SPI_WRITE(d) do { hspi1.Instance->DR = d; } while(0)

/** Macro for reading the radio SPI data register.
 *
 */
#define HAL_NRF_HW_SPI_READ() hspi1.Instance->DR

/** Macro specifyng the radio SPI busy flag.
 *
 */
#define HAL_NRF_HW_SPI_BUSY (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET)

/**
 * Pulses the CE to nRF24L01 for at least 10 us
 */
#define CE_PULSE() \
    do { \
        CE_HIGH();  \
        osDelay(1); \
        CE_LOW();  \
    } while(0)

#endif /* HAL_NRF_HW_H__ */