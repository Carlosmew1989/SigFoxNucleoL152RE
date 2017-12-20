/**
* @file    st_lowlevel_utils.h
* @author  LowPower RF BU - AMG
* @version 1.3.0
* @date    March 1, 2017
* @brief   This contains some utilities related to the specific implementation of the st_lowlevel module.
* @details
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
*/


#ifndef ST_LOWLEVEL_UTILS_H_
#define ST_LOWLEVEL_UTILS_H_

#include "stdint.h"

/*!
 * \defgroup ST_LOWLEVEL_UTILS
 *
 *  @{
 */
   

/*!******************************************************************
 * \fn void ST_LOWLEVEL_UTILS_SetSysClock(void)
 * \brief  This function is used to confgure the system clock when the STM32 exits
 *  the low power or at beginning of the application.
 * \note This is a function that is not required by the ST-MANUF library.
 * \param[in] None.
 * \retval None.
 *******************************************************************/
void ST_LOWLEVEL_UTILS_SetSysClock(void);

/*!******************************************************************
 * \fn void ST_LOWLEVEL_UTILS_TimerCalibration(uint16_t duration_ms)
 * \brief  This function calibrates the RTC that is used by the st_lowlevel when the
 *              device goes in sleep.
 * \note This is a function that is not required by the ST-MANUF library.
* \param[in] uint16_t duration_ms : duration of the calibration process in ms.
 * \retval None.
 *******************************************************************/
void ST_LOWLEVEL_UTILS_TimerCalibration(uint16_t duration_ms);

/*!******************************************************************
 * \fn void ST_LOWLEVEL_UTILS_LowPower(uint8_t en)
 * \brief  This function instructs the st_lowlevel to send the STM32 in sleep or not
 *              during the protocol operations.
 *              It is mainly used for debugging purposes.
 * \note This is a function that is not required by the ST-MANUF library.
* \param[in] uint8_t en : enable the low power (1, default setting) or not (0).
 * \retval None.
 *******************************************************************/
void ST_LOWLEVEL_UTILS_LowPower(uint8_t en);

/** @}*/

#endif /* ST_LOWLEVEL_UTILS_H_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
