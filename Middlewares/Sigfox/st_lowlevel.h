/**
* @file    st_lowlevel.h
* @author  LowPower RF BU - AMG
* @version 1.4.0
* @date    April 28, 2017
* @brief   This is the interface of the low level functions to be implemented for the platform used.
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


#ifndef ST_LOWLEVEL_H_
#define ST_LOWLEVEL_H_

#include "stdint.h"

/*!
 * \defgroup ST_LOWLEVEL
 *
 *  @{
 */
   
   
/*!******************************************************************
 * \fn void ST_LOWLEVEL_Shutdown(uint8_t value)
 * \brief Set on or off the S2-LP by GPIO.
 * \param[in] uint8_t value: 
 *     if 1, the device should enter shutdown (OFF).
 *     if 0, the device should exit from shutdown (ON).
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_Shutdown(uint8_t value);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_Delay(uint32_t delay_ms)
 * \brief Wait delay_ms milliseconds in a blocking way.
 * \param[in] uint32_t delay_ms: the number of milliseconds to wait.
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_Delay(uint32_t delay_ms);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_SpiRaw(uint8_t n_bytes, uint8_t* in_buffer, uint8_t* out_buffer, uint8_t can_return_bef_tx)
 * \brief Performs a raw SPI operation with the passed input buffer and stores the returned SPI bytes in the output buffer.
 * \param[in] uint8_t n_bytes: number of elements of the total SPI transaction
 * \param[in] uint8_t *in_buffer: pointer to the input buffer (uC memory where the SPI data to write are stored).
 * \param[in] uint8_t *out_buffer: pointer to the output buffer (the uC memory where the data from SPI must be stored).
 * \param[in] uint8_t can_return_bef_tx: if this flag is 1, it means that the function can be non-blocking returning immediately.
 *             Sometimes, the library asks to the low_level to transfer a big amount of data to the SPI in order to generate
 *              the BPSK modulation.
 *              When this flag is 0, the ST_LOWLEVEL_SpiRaw function should not return until the 
 *              SPI transaction is completed (using a DMA that performs the transaction on its own).
 *             This is intended for those applications that want to use the CPU during long SPI transactions.
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_SpiRaw(uint8_t n_bytes, uint8_t* in_buffer, uint8_t* out_buffer, uint8_t can_return_bef_tx);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_TimerStart(uint32_t time_duration_ms)
 * \brief This function starts a timer without blocking the application. It is recommended to use an RTC to allow the uC to go in low power.
 * \param[in] uint32_t time_duration_ms: the total duration of the timer in milliseconds.
 * \retval None
 * \note When this timer expires, the callaback \ref ST_MANUF_Timer_CB should be called.
 *      This function is implemented by the ST-MANUF library.
 *******************************************************************/
void ST_LOWLEVEL_TimerStart(uint32_t time_duration_ms);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_TimerStop(void)
 * \brief This function stops the timer started by the \ref ST_LOWLEVEL_TimerStart .
 * \param[in] None.
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_TimerStop(void);


/*!******************************************************************
 * \fn uint32_t ST_LOWLEVEL_TimerGetExpMs(void)
 * \brief This function returns the number of ms from when the timer was started by the \ref ST_LOWLEVEL_TimerStart .
 * \param[in] None.
 * \retval uint32_t milliseconds read from the timer.
 * \note This function is used only by the library for ARIB (RCZ3) to implement the LBT mechanism.
 *******************************************************************/
uint32_t ST_LOWLEVEL_TimerGetExpMs(void);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_Encrypt(uint8_t *encrypted_data, uint8_t *data_to_encrypt, uint8_t data_len)
 * \brief This function is in charge of encrypting the data passed by the SigFox library.
 * \param[in] uint8_t *encrypted_data: pointer to the destination buffer where the encrypted data must be stored.
 * \param[in] uint8_t *data_to_encrypt: pointer to the source buffer where the data to encrypt are stored.
 * \param[in] uint8_t data_len: length of the data buffer to encrypt.
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_Encrypt(uint8_t *encrypted_data, uint8_t *data_to_encrypt, uint8_t data_len);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_NVMRead(uint32_t offset)
 * \brief This function is used to read a 16bits word from the NVM used by the SigFox library.
 * \param[in] uint32_t offset: the offset from the base address of the NVM to read.
 * \retval uint16_t half-word read from he NVM.
 *******************************************************************/
uint16_t ST_LOWLEVEL_NVMRead(uint32_t offset);

/*!******************************************************************
 * \fn void ST_LOWLEVEL_NVMWrite(uint32_t offset, uint32_t value)
 * \brief This function is used to write a 16bits word from the NVM used by the SigFox library.
 * \param[in] uint32_t offset: the offset from the base address of the NVM to write.
 * \param[in] uint32_t value: the data to write.
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_NVMWrite(uint32_t offset, uint32_t value);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_GpioIRQ(uint8_t pin, uint8_t new_state, uint8_t trigger)
 * \brief Enables or Disables the external interrupt on the microcontroller side. The interrupt must be set on the rising or falling edge of the input signal according to the trigger_flag.
 *  The pin number passed represents the GPIO number of the S2-LP.
 * \param[in] uint8_t pin: the GPIO pin of the S2-LP (integer from 0 to 3).
 * \param[in] uint8_t new_state: enable or disable the EXTI (can be 0 or 1).
 * \param[in] uint8_t trigger: trigger_flag:
 *                   1: rising edge
 *                   0: falling edge
 * \retval None
 *******************************************************************/
void ST_LOWLEVEL_GpioIRQ(uint8_t pin, uint8_t new_state, uint8_t trigger);


/*!******************************************************************
 * \fn void ST_LOWLEVEL_WaitForInterrupt(void)
 * \brief Microcontroller waits for interrupt.
 *      This function is continously called by the library each time it waits for an event.
 *      This is useful if the application must trigger a state machine and not block the CPU waiting for an event from the library.
 *      For example, this can be a null implementation or can activate a low power mode of the microcontroller, tick a stack for
 *      dual radio applications or other type of state machines.
 * \param[in] None.
 * \retval None.
 *******************************************************************/
void ST_LOWLEVEL_WaitForInterrupt(void);


/*!******************************************************************
 * \fn int32_t ST_LOWLEVEL_GetTemperature(void)
 * \brief Returns the temperature in 1/10 of Celsius degree. If not possible return 0.
 * \param[in] None.
 * \retval int32_t temperature in deg C/10.
 *******************************************************************/
int32_t ST_LOWLEVEL_GetTemperature(void);


/*!******************************************************************
 * \fn int32_t ST_LOWLEVEL_GetVoltage(void)
 * \brief Returns the voltage in mV. If not possible return 0.
 * \param[in] None.
 * \retval int32_t voltage in mV.
 *******************************************************************/
int32_t ST_LOWLEVEL_GetVoltage(void);


/*!******************************************************************
 * \fn void ST_MANUF_Timer_CB(void)
 * \brief This is a <b>callback</b> exported by the ST-MANUF library. 
 *      It must be called when the timer started by \ref ST_LOWLEVEL_TimerStart expires.
 * \note This function mustn't be implemented by the st_lowlevel module because 
 *      it is already implemented by the ST-MANUF library.
 * \param[in] None.
 * \retval None.
 *******************************************************************/
void ST_MANUF_Timer_CB(void);


/*!******************************************************************
 * \fn void ST_MANUF_S2LP_Exti_CB(void)
 * \brief This is a <b>callback</b> exported by the ST-MANUF library.
 *      The ST-MANUF module configures the S2-LP to raise interrupts and to notify them 
 *      on a GPIO using the (\ref ST_LOWLEVEL_GpioIRQ ). When the interrupt of that GPIO 
 *      is raised, this function must be called.
 *      It must be called when the S2-LP raises the IRQ via GPIO.
 * \note This function mustn't be implemented by the st_lowlevel module because 
 *      it is already implemented by the ST-MANUF library.
 * \param[in] None.
 * \retval None.
 *******************************************************************/
void ST_MANUF_S2LP_Exti_CB(void);



/** @}*/

#endif /* ST_LOWLEVEL_H_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
