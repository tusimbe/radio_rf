/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic 
 * Semiconductor ASA.Terms and conditions of usage are described in detail 
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *              
 * $LastChangedRevision: 133 $
 */

/** @file
 * @brief Gazell Link Layer nRF24LE1 specific macros
 */
 
/** @ingroup nordic_gzll */

/** @name Hardware dependent macros for the Gazell Link Layer.  
@{
*/

#ifndef GZLL_MACROS_H__
#define GZLL_MACROS_H__

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;

/** @brief Defines the radio interrupt enable bit*/
#define RF_INT_ENABLE()   __HAL_GPIO_EXTI_ENABLE_IT(GPIO_PIN_1)
#define RF_INT_DISABLE()  __HAL_GPIO_EXTI_DISABLE_IT(GPIO_PIN_1)

/** @brief Defines the timer interrupt enable bit*/
#define TIMER_INT_ENABLE()  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
#define TIMER_INT_DISABLE() __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);

#endif // GZLL_MACROS_H__

/** @} */

