/**
* @file    st_sigfox_api.h
* @author  LowPower RF BU - AMG
* @version 1.4.0
* @date    April 28, 2017
* @brief   This is the interface to the application.
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

#ifndef ST_SFX_API_H_
#define ST_SFX_API_H_

#include <stdint.h>

/*!
 * \defgroup ST_SIGFOX_API
 *
 *  @{
 */

/* Custom Types */
typedef uint16_t      st_sfx_error_t;

/*!
 * \defgroup SFX_ERR_CODES Return Error codes definition for SIGFOX APIs
 *
 *  @{
 */

#define SFX_ERR_NONE                          0x00 /*!< No error */

#define SFX_ERR_OPEN_MALLOC                   0x10 /*!< Error on MANUF_API_malloc or buffer pointer NULL */
#define SFX_ERR_OPEN_ID_PTR                   0x11 /*!< ID pointer NULL */
#define SFX_ERR_OPEN_GET_SEQ                  0x12 /*!< Error on MANUF_API_get_nv_mem w/ SFX_NVMEM_SEQ_CPT */
#define SFX_ERR_OPEN_GET_PN                   0x13 /*!< Error on MANUF_API_get_nv_mem w/ SFX_NVMEM_PN */
#define SFX_ERR_OPEN_STATE                    0x14 /*!< State is not idle, library should be closed before */
#define SFX_ERR_OPEN_GET_FH                   0x15 /*!< Error on MANUF_API_get_nv_mem w/ SFX_NVMEM_FH */
#define SFX_ERR_OPEN_MACRO_CHANNEL_WIDTH      0x16 /*!< Macro channel width not allowed */
#define SFX_ERR_OPEN_RCZ_PTR                  0x17 /*!< RCZ pointer is NULL */

#define SFX_ERR_CLOSE_FREE                    0x20 /*!< Error on MANUF_API_free */
#define SFX_ERR_CLOSE_RF_STOP                 0x21 /*!< Error on MANUF_API_rf_stop */

#define SFX_ERR_SEND_FRAME_DATA_LENGTH        0x30 /*!< Customer data length > 12 Bytes */
#define SFX_ERR_SEND_FRAME_STATE              0x31 /*!< State != READY, must close and reopen library */
#define SFX_ERR_SEND_FRAME_RESPONSE_PTR       0x32 /*!< Response data pointer NULL in case of downlink */
#define SFX_ERR_SEND_FRAME_BUILD_UPLINK       0x33 /*!< Build uplink frame failed */
#define SFX_ERR_SEND_FRAME_SEND_UPLINK        0x34 /*!< Send uplink frame failed */
#define SFX_ERR_SEND_FRAME_RECEIVE            0x35 /*!< Receive downlink frame failed or timeout */
#define SFX_ERR_SEND_FRAME_DELAY_OOB_ACK      0x36 /*!< Error on MANUF_API_delay w/ SFX_DLY_OOB_ACK (Downlink) */
#define SFX_ERR_SEND_FRAME_BUILD_OOB_ACK      0x37 /*!< Build out of band frame failed (Downlink) */
#define SFX_ERR_SEND_FRAME_SEND_OOB_ACK       0x38 /*!< Send out of band frame failed (Downlink) */
#define SFX_ERR_SEND_FRAME_DATA_PTR           0x39 /*!< Customer data pointer NULL */
#define SFX_ERR_SEND_FRAME_CARRIER_SENSE_CONFIG  0x3A /*!< Carrier Sense configuration need to be initialized */
#define SFX_ERR_SEND_FRAME_CARRIER_SENSE_TIMEOUT 0x3B /*!< Wait for clear channel has returned time out */
#define SFX_ERR_SEND_FRAME_WAIT_TIMEOUT       0x3E /*!< Wait frame has returned time out */
#define SFX_ERR_SEND_FRAME_INVALID_FH_CHAN    0x3F /*!< FH invalid channel, must call SIGFOX_API_reset */


#define SFX_ERR_SEND_BIT_STATE                0x41 /*!< State != READY, must close and reopen library */
#define SFX_ERR_SEND_BIT_RESPONSE_PTR         0x42 /*!< Response data pointer NULL in case of downlink */
#define SFX_ERR_SEND_BIT_BUILD_UPLINK         0x43 /*!< Build uplink frame failed */
#define SFX_ERR_SEND_BIT_SEND_UPLINK          0x44 /*!< Send uplink frame failed */
#define SFX_ERR_SEND_BIT_RECEIVE              0x45 /*!< Receive downlink frame failed or timeout */
#define SFX_ERR_SEND_BIT_DELAY_OOB_ACK        0x46 /*!< Error on MANUF_API_delay w/ SFX_DLY_OOB_ACK (Downlink) */
#define SFX_ERR_SEND_BIT_BUILD_OOB_ACK        0x47 /*!< Build out of band frame failed (Downlink) */
#define SFX_ERR_SEND_BIT_SEND_OOB_ACK         0x48 /*!< Send out of band frame failed (Downlink) */
#define SFX_ERR_SEND_BIT_DATA_PTR             0x49 /*!< Customer data pointer NULL */
#define SFX_ERR_SEND_BIT_CARRIER_SENSE_CONFIG  0x4A /*!< Carrier Sense configuration need to be initialized */
#define SFX_ERR_SEND_BIT_CARRIER_SENSE_TIMEOUT 0x4B /*!< Wait for clear channel has returned time out */
#define SFX_ERR_SEND_BIT_WAIT_TIMEOUT         0x4E /*!< Wait frame has returned time out */
#define SFX_ERR_SEND_BIT_INVALID_FH_CHAN      0x4F /*!< FH invalid channel, must call SIGFOX_API_reset */

#define SFX_ERR_SEND_OOB_STATE                0x51 /*!< State != READY, must close and reopen library */
#define SFX_ERR_SEND_OOB_BUILD_UPLINK         0x53 /*!< Build uplink frame failed */
#define SFX_ERR_SEND_OOB_SEND_UPLINK          0x54 /*!< Send uplink frame failed */
#define SFX_ERR_SEND_OOB_INVALID_FH_CHAN      0x5F /*!< Send out of band frame failed (Downlink) */

/* 0x006x to 0x008x codes available */

#define SFX_ERR_SET_STD_CONFIG_SIGFOX_CHAN    0x90 /*!< Default SIGFOX channel out of range */
#define SFX_ERR_SET_STD_CONFIG_SET            0x91 /*!< Unable to set configuration */


#define SFX_ERR_TEST_MODE_0_RF_INIT           0xA0 /*!< Error on MANUF_API_rf_init */
#define SFX_ERR_TEST_MODE_0_CHANGE_FREQ       0xA1 /*!< Error on MANUF_API_change_frequency */
#define SFX_ERR_TEST_MODE_0_RF_SEND           0xA2 /*!< Error on MANUF_API_rf_send */
#define SFX_ERR_TEST_MODE_0_DELAY             0xA3 /*!< Error on MANUF_API_delay */
#define SFX_ERR_TEST_MODE_0_RF_STOP           0xA4 /*!< Error on MANUF_API_rf_stop */

#define SFX_ERR_TEST_MODE_STATE               0xB1 /*!< State != READY, must close and reopen library */

#define SFX_ERR_TEST_MODE_2_REPORT_TEST       0xC0 /*!< Error on MANUF_API_report_test_result */

#define SFX_ERR_TEST_MODE_3_RF_INIT           0xD0 /*!< Error on MANUF_API_rf_init */
#define SFX_ERR_TEST_MODE_3_CHANGE_FREQ       0xD1 /*!< Error on MANUF_API_change_frequency */
#define SFX_ERR_TEST_MODE_3_TIMER_START       0xD2 /*!< Error on MANUF_API_timer_start */
#define SFX_ERR_TEST_MODE_3_REPORT_TEST       0xD3 /*!< Error on MANUF_API_report_test_result */
#define SFX_ERR_TEST_MODE_3_TIMER_STOP        0xD4 /*!< Error on MANUF_API_timer_stop */
#define SFX_ERR_TEST_MODE_3_RF_STOP           0xD5 /*!< Error on MANUF_API_rf_stop */

#define SFX_ERR_TEST_MODE_4_BUILD_UPLINK      0xE0 /*!< Build uplink frame failed */
#define SFX_ERR_TEST_MODE_4_SEND_UPLINK       0xE1 /*!< Send uplink frame failed */
#define SFX_ERR_TEST_MODE_4_REPORT_TEST       0xE2 /*!< Error on MANUF_API_report_test_result */
#define SFX_ERR_TEST_MODE_4_GET_RSSI          0xE3 /*!< Error on MANUF_API_get_rssi */
#define SFX_ERR_TEST_MODE_4_DELAY             0xE4 /*!< Error on MANUF_API_delay */

#define SFX_ERR_TEST_MODE_5_RF_INIT           0xF0 /*!< Error on MANUF_API_rf_init */
#define SFX_ERR_TEST_MODE_5_CHANGE_FREQ       0xF1 /*!< Error on MANUF_API_change_frequency */
#define SFX_ERR_TEST_MODE_5_BUILD_UPLINK      0xF2 /*!< Build uplink frame failed */
#define SFX_ERR_TEST_MODE_5_SEND_UPLINK       0xF3 /*!< Send uplink frame failed */
#define SFX_ERR_TEST_MODE_5_RF_STOP           0xF4 /*!< Error on MANUF_API_rf_stop */

/* ST defined codes */

#define ST_SIGFOX_TEST_MODE_ERROR     0x80FC   /*!< Test mode not admitted */
#define ST_SIGFOX_GPIO_ERROR          0x80FD   /*!< GPIO error */
#define ST_SIGFOX_POWER_ERROR         0x80FE   /*!< Power error */
#define ST_SIGFOX_RCZ_ERROR           0x80FF   /*!< RCZ not permitted */


/** @}*/

/*!*******************************
 * \enum st_rcz_t
 * \brief Define all the RCZ
 *******************************/
typedef enum
{
  ST_RCZ1=1,    /*!< Radio Configuration zone 1 */
  ST_RCZ2=2,    /*!< Radio Configuration zone 2 */
  ST_RCZ3=3,    /*!< Radio Configuration zone 3 (not supported yet) */
  ST_RCZ4=4,    /*!< Radio Configuration zone 4 */
}st_rcz_t;


/*!*******************************
 * \enum st_sfx_test_mode_t
 * \brief Define all the test mode
 *******************************/
typedef enum
{
    ST_SFX_TEST_MODE_TX_BPSK     = 0,  /*!< only BPSK with Synchro Bit + Synchro frame + PN sequence : no hopping centered on the TX_frequency */
    ST_SFX_TEST_MODE_TX_PROTOCOL = 1,  /*!< with full protocol defined at ST_SIGFOX_API_open call: send all SIGFOX protocol frames available with hopping */
    ST_SFX_TEST_MODE_RX_PROTOCOL = 2,  /*!< with full protocol defined at ST_SIGFOX_API_open call: send SIGFOX protocol frames w/ initiate downlink flag = SFX_TRUE */
    ST_SFX_TEST_MODE_RX_GFSK     = 3,  /*!< with known pattern with SB + SF + Pattern on RX_Frequency defined at SIGFOX_API_open function : od internaly compare received frame <=> known pattern and call void ST_MANUF_report_CB(uint8_t status, int32_t rssi) */
    ST_SFX_TEST_MODE_RX_SENSI    = 4,  /*!< Do uplink +  downlink frame defined at ST_SIGFOX_API_open call but specific shorter timings */
    ST_SFX_TEST_MODE_TX_SYNTH    = 5,  /*!< Do 1 uplink frame on each sigfox channel to measure frequency synthesis step */
} st_sfx_test_mode_t;



/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_open(st_rcz_t rcz, uint8_t *id_ptr)
 * \brief This function opens the library initializing all the state machine parameters.
 *      This function does not involve the radio configuration.
 * \param[in] st_rcz_t rcz: RCZ number. It can be one of the values of \ref st_rcz_t.
 * \param[in] uint8_t *id_ptr: pointer to 32bits word containing the ID (casted as uint8_t*).
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_open(st_rcz_t rcz, uint8_t *id_ptr);


/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_close(void)
 * \brief Closes the sigfox library resetting its state.
 * \param[in] None.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_close(void);


/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_send_frame(uint8_t *customer_data,
                                  uint8_t customer_data_length,
                                  uint8_t *customer_response,
                                  uint8_t tx_repeat,
                                  uint8_t initiate_downlink_flag)
 * \brief Send a frame to the sigfox newtork.
 * \param[in] uint8_t *customer_data: pointer to the data to transmit.
 * \param[in] uint8_t customer_data_length: size in bytes of the data to transmit (max 12).
 * \param[in] uint8_t *customer_response: pointer to the buffer where to store the received payload (only if initiate_downlink_flag=1, see below).
 * \param[in] uint8_t tx_repeat: number of repetitions (only if initiate_downlink_flag=1, see below)
 * \param[in] uint8_t initiate_downlink_flag: wait for a response after transmitting.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_send_frame(uint8_t *customer_data,
                                  uint8_t customer_data_length,
                                  uint8_t *customer_response,
                                  uint8_t tx_repeat,
                                  uint8_t initiate_downlink_flag);


/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_send_bit(uint8_t bit_value,
                                uint8_t *customer_response,
                                uint8_t tx_repeat,
                                uint8_t initiate_downlink_flag)
 * \brief This function is used to send a single bit. It is mainly used when the node is interested to downlink data (and doesn't want to transmit).
 * \param[in] uint8_t bit_value: bit value to send.
 * \param[in] uint8_t *customer_response: pointer to the buffer where to store the received payload (only if initiate_downlink_flag=1, see below).
 * \param[in] uint8_t tx_repeat: number of repetitions (only if initiate_downlink_flag=1, see below)
 * \param[in] uint8_t initiate_downlink_flag: wait for a response after transmitting.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_send_bit(uint8_t bit_value,
                                uint8_t *customer_response,
                                uint8_t tx_repeat,
                                uint8_t initiate_downlink_flag);



/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_reset(void)
 * \brief Reset the library state.
 * \param[in] None.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_reset(void);


/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_set_std_config(uint32_t config_words[3],
                                      uint16_t default_sigfox_channel)
 * \brief Configure the config_words in order to use a proper channel map.
 * \param[in] uint32_t config_words[3]: 3 config words array to select the FCC channels to use
 * \param[in] uint16_t default_sigfox_channel: default channel to be used among those selected by the config_words.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_set_std_config(uint32_t config_words[3],
                                      uint16_t default_sigfox_channel);


/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_get_std_config(uint32_t config_words[3],
                                      uint16_t *default_sigfox_channel_ptr)
 * \brief Get the the config_words configured. These returned words are valid only for FCC
 * \param[in] uint32_t config_words[3]: 3 config words array to select the FCC channels to use
 * \param[in] uint16_t *default_sigfox_channel_ptr: pointer to the variable where the default channel is stored.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_get_std_config(uint32_t config_words[3],
                                      uint16_t *default_sigfox_channel_ptr);


/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_get_version(uint8_t** version,
                                                uint8_t* size)
 * \brief Returns the library version.
 * \param[in] uint8_t **version: pointer to the array where to store the lib version.
 * \param[in] uint8_t *size: size of the written version array.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_get_version(uint8_t** version,
                                   uint8_t* size);



/*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_get_info(uint8_t* returned_info)
 * \brief Returns the library version.
 * \param[in] uint8_t *returned_info containing info about the standard configuration.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_get_info(uint8_t* returned_info);

/*!******************************************************************
 * \fn void ST_MANUF_API_get_version(uint8_t *ver, uint8_t *size)
 * \brief Returns the version of the MANUF_API implemented by ST.
 * \param[in] uint8_t *ver: an array containing the library version.
 *              For example, [1,0,0] means version 1.0.0
 * \param[in] uint8_t *size: a pointer where to store the version array size.
 * \retval None.
 *******************************************************************/
void ST_MANUF_API_get_version(uint8_t *ver, uint8_t *size);

/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_gpio_irq_pin(uint8_t gpio_pin)
 * \brief Configures one of the S2-LP pin to be an IRQ pin.
 * \param[in] uint8_t gpio_pin: an integer in the range [0,3] representing the GPIO to be used as IRQ.
 *                  Default value is 3.
 * \note This function must be called before \ref ST_SIGFOX_API_open .
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_gpio_irq_pin(uint8_t gpio_pin);

/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_gpio_tx_rx_pin(uint8_t gpio_pin)
 * \brief Configures one of the S2-LP pin to be to be configured as (RX or TX) signal.
 * \param[in] uint8_t gpio_pin: an integer in the range [0,3] representing the GPIO to be configured as (RX or TX) signal.
 *                  Default value is 0.
 * \note Only for RCZ2/4. Uneffective for RCZ1. This function must be called before \ref ST_SIGFOX_API_open .
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_gpio_tx_rx_pin(uint8_t gpio_pin);

/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_gpio_rx_pin(uint8_t gpio_pin)
 * \brief Configures one of the S2-LP pin to be configured as RX signal.
 * \param[in] uint8_t gpio_pin: an integer in the range [0,3] representing the GPIO to be configured as RX signal.
 *                  Default value is 1.
 * \note Only for RCZ2/4. Uneffective for RCZ1. This function must be called before \ref ST_SIGFOX_API_open .
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_gpio_rx_pin(uint8_t gpio_pin);


/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_gpio_tx_pin(uint8_t gpio_pin)
 * \brief Configures one of the S2-LP pin to be configured as TX signal.
 * \param[in] uint8_t gpio_pin: an integer in the range [0,3] representing the GPIO to be configured as TX signal.
 *                  Default value is 2.
 * \note Only for RCZ2/4. Uneffective for RCZ1. This function must be called before \ref ST_SIGFOX_API_open .
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_gpio_tx_pin(uint8_t gpio_pin);


/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_reduce_output_power(int16_t reduction)
 * \brief Reduces the output power of the transmitted signal by a facor (reduction*0.5dB against the actual value).
 * \details       Each positive step of 1 reduces the power at S2-LP level of about 0.5dB. A negative value increase the power level of the same quantity.
 *        <br>The function returns an error if the output power is bigger than the one used dutring cerification.
 * \param[in] uint8_t reduction: the reduction factor.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_reduce_output_power(int16_t reduction);


/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_set_xtal_freq(uint32_t xtal)
 * \brief Sets the XTAL frequency of the S2-LP in Hertz (default is 50MHz).
 * \param[in] uint32_t xtal: the xtal frequency of the S2-LP in Hz as an integer.
 * \note If this function is not called, the default xtal frequency is 50MHz.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_set_xtal_freq(uint32_t xtal);


/*!******************************************************************
 * \fn st_sfx_error_t ST_MANUF_API_set_freq_offset(int32_t offset)
 * \brief Sets the RF frequency offset in Hertz (default is 0 Hz).
 * \param[in] int32_t offset: frequency offset in Hz as an integer.
 * \note If this function is not called, the default frequency offset is 0 Hz.
 * \retval 0 if no error, an error code from the /ref SFX_ERR_CODES otherwise.
 *******************************************************************/
st_sfx_error_t ST_MANUF_API_set_freq_offset(int32_t offset);



 /*!******************************************************************
 * \fn st_sfx_error_t ST_SIGFOX_API_test_mode(st_sfx_test_mode_t test_mode, uint8_t config)
 * \brief This function is used for protocol/RF/sensitivity tests.
 * All tests must be accessible for UNB modem Qualification P1.
 *
 *  - A1 <B>If test_mode = ST_SFX_TEST_MODE_TX_BPSK (TEST_MODE_0).</B><BR>
 *  This test consists in sending PRBS data in a 26 Bytes frame @ constant frequency.<BR>
 *  Bit 7 of config defines if a delay is applied of not in the loop (continuous
 *  modulated wave mode for power measuring).<BR>
 *  This test is used to check :
 *      - 1 Spectrum analysis
 *      - 2 Modulation bit rate
 *      - 3 Modulation phase
 *      - 4 Ramp up sequence
 *      - 5 Ramp down sequence
 *      - 6 Dynamic drift
 *      .
 *  - A2 <B>ST_SFX_TEST_MODE_TX_BPSK Schedule :</B>
 *      - 1 Init RF for transmission
 *      - 2 Change synthesizer frequency @ tx_frequency configured in ST_SIGFOX_API_open
 *      - 3 Loop 0 to (config & 0x7F) (input argument of function)
 *          - 3a Build a 26 Bytes frame based on PRBS generator
 *          - 3b Send it over RF
 *          - 3c (if bit 7 of config == 0) Apply interframe (TRX) delay else no delay
 *          .
 *      - 4 End Loop
 *      - 5 Close RF
 *      .
 *  .
 *  ______________________________________________________________________________________________
 *  - B1 <B>If test_mode = ST_SFX_TEST_MODE_TX_PROTOCOL (TEST_MODE_1).</B><BR>
 *  This test consists in calling send functions to test the
 *  complete protocol in Uplink only.<BR>
 *  This test is used to check :
 *      - 1 Frames w/ 1 Byte to 12 Bytes payloads
 *      - 2 Bit frames
 *      - 3 Out of band frame (and Voltage + Temperature consistency in payload)
 *      - 4 Sequence number saving
 *      - 5 Frequency hopping
 *      - 6 Static drift
 *      - 7 Interframe delay
 *      .
 *  - B2 <B>ST_SFX_TEST_MODE_TX_PROTOCOL Schedule :</B>
 *      - Loop 0 to config (input argument of function)
 *          - 1 Call SIGFOX_API_send_bit w/ bit_value = FALSE
 *          - 2 Call SIGFOX_API_send_bit w/ bit_value = TRUE
 *          - 3 Call SIGFOX_API_send_outofband
 *          - 4 Loop 1 to 12
 *              - 4a Call SIGFOX_API_send_frame w/ customer_data_length = loop. Data = 0x30 to 0x3B
 *              .
 *          .
 *      - End Loop
 *      .
 *  .
 *  ______________________________________________________________________________________________
 *  - C1 <B>If test_mode = ST_SFX_TEST_MODE_RX_PROTOCOL (TEST_MODE_2).</B><BR>
 *  This test consists in calling ST_SIGFOX_API_send_xxx functions to test the
 *  complete protocol in Downlink only.<BR>
 *  This test is used to check :
 *      - 1 Downlink frames
 *      - 2 Interframe delay
 *      - 3 Out of band frame (and Voltage + Temperature + RSSI consistency in payload)
 *      - 4 20 seconds wait window
 *      - 5 25 seconds listening window
 *      - 6 Delay between ACK and OOB frame
 *      .
 *  - C2 <B>ST_SFX_TEST_MODE_RX_PROTOCOL Schedule :</B>
 *      - Loop 0 to config (input argument of function)
 *          - 1 Call the send frame function w/ customer_data_length = 12, Data = 0x30 to 0x3B,
 *          downlink_flag = SFX_TRUE, tx_repeat = 2.
 *          .
 *      - End Loop
 *      .
 *  .
 *  ______________________________________________________________________________________________
 *  - D1 <B>If test_mode = ST_ST_SFX_TEST_MODE_RX_GFSK (TEST_MODE_3).</B><BR>
 *  This test consists in receiving constant GFSK frames @ constant frequency.<BR>
 *  The pattern used for test is : <B>AA AA B2 27 1F 20 41 84 32 68 C5 BA 53 AE 79 E7 F6 DD 9B</B>
 *  with <B>AA AA B2 27</B> configured in RF chip<BR>
 *  This test is used to check :
 *      - 1 GFSK receiver
 *      - 2 Approximative sensitivity
 *      .
 *  - D2 <B>ST_SFX_TEST_MODE_RX_GFSK Schedule :</B>
 *      - 1 Init RF in reception mode
 *      - 2 Change synthesizer frequency @ rx_frequency configured in ST_SIGFOX_API_open
 *      - 3 Start timer w/ config in seconds (input argument of function)
 *      - 4 While MANUF_API_wait_frame does not return TIME_OUT
 *          - 4a Read data from RF chip
 *          - 4b Compare w/ constant pattern (above)
 *          - 4b If OK (pattern == received_data), call report_test_result TRUE else FALSE
 *          .
 *      - 5 End Loop
 *      - 6 Stop timer
 *      - 7 Close RF
 *      .
 *  .
 *  ______________________________________________________________________________________________
 *  - E1 <B>If test_mode = ST_SFX_TEST_MODE_RX_SENSI (TEST_MODE_4).</B><BR>
 *  This test is specific to SIGFOX's test equipments & softwares.<BR>
 *  It is mandatory to measure the real sensitivity of device.<BR>
 *  This test is used to check :
 *      - 1 Device sensitivity
 *      .
 *  - E2 <B>ST_SFX_TEST_MODE_RX_SENSI Schedule :</B>
 *      - Loop 0 to (config x 10) (input argument of function)
 *          - 1 Send 1 Uplink Frame @ tx_frequency + hopping
 *          - 2 Receive 1 Downlink Frame @ rx_frequency + hopping
 *          - 3 If OK, call report_test_result TRUE else FALSE
 *          - 4 Delay
 *          .
 *      - End Loop
 *      .
 *  .
 *  ______________________________________________________________________________________________
 *
 *  - F1 <B>If test_mode = ST_SFX_TEST_MODE_TX_SYNTH (TEST_MODE_5).</B><BR>
 *  This test consists in sending SIGFOX frames with 4Bytes payload @ forced frequency.<BR>
 *  This test is used to check :
 *      - 1 Synthetiser's frequency step
 *      - 2 Static drift
 *      .
 *  - F2 <B>ST_SFX_TEST_MODE_TX_SYNTH Schedule :</B>
 *      - 1 Init RF for transmission
 *      - 2 Loop 0 to Maximum number of 100 Hz channels
 *          - 2a Change synthesizer frequency @ forced frequency
 *          - 2b Build a 4 Bytes payload frame w/ forced frequency value
 *          - 2c Send it over RF
 *          .
 *      - 3 End Loop
 *      - 4 Close RF
 *      .
 *  .
 *
 * \param[in] st_sfx_test_mode_t test_mode         Test mode selection
 * \param[in] uint8_t config                       The use of config depends on test mode (see above)
 *
 * \retval SFX_ERR_TEST_MODE_STATE:               State != READY, must close and reopen library
 * \retval SFX_ERR_TEST_MODE_0_RF_INIT:           Error on MANUF_API_rf_init
 * \retval SFX_ERR_TEST_MODE_0_CHANGE_FREQ:       Error on MANUF_API_change_frequency
 * \retval SFX_ERR_TEST_MODE_0_RF_SEND:           Error on MANUF_API_rf_send
 * \retval SFX_ERR_TEST_MODE_0_DELAY:             Error on MANUF_API_delay
 * \retval SFX_ERR_TEST_MODE_0_RF_STOP:           Error on MANUF_API_rf_stop
 * \retval SFX_ERR_TEST_MODE_2_REPORT_TEST:       Error on MANUF_API_report_test_result
 * \retval SFX_ERR_TEST_MODE_3_RF_INIT:           Error on MANUF_API_rf_init
 * \retval SFX_ERR_TEST_MODE_3_CHANGE_FREQ:       Error on MANUF_API_change_frequency
 * \retval SFX_ERR_TEST_MODE_3_TIMER_START:       Error on MANUF_API_timer_start
 * \retval SFX_ERR_TEST_MODE_3_WAIT_FRAME:        Error on MANUF_API_wait_frame
 * \retval SFX_ERR_TEST_MODE_3_REPORT_TEST:       Error on MANUF_API_report_test_result
 * \retval SFX_ERR_TEST_MODE_3_TIMER_STOP:        Error on MANUF_API_timer_stop
 * \retval SFX_ERR_TEST_MODE_3_RF_STOP:           Error on MANUF_API_rf_stop
 * \retval SFX_ERR_TEST_MODE_4_BUILD_UPLINK:      Build uplink frame failed
 * \retval SFX_ERR_TEST_MODE_4_SEND_UPLINK:       Send uplink frame failed
 * \retval SFX_ERR_TEST_MODE_4_REPORT_TEST:       Error on MANUF_API_report_test_result
 * \retval SFX_ERR_TEST_MODE_4_GET_RSSI:          Error on MANUF_API_get_rssi
 * \retval SFX_ERR_TEST_MODE_4_DELAY:             Error on MANUF_API_delay
 * \retval SFX_ERR_TEST_MODE_5_RF_INIT:           Error on MANUF_API_rf_init
 * \retval SFX_ERR_TEST_MODE_5_CHANGE_FREQ:       Error on MANUF_API_change_frequency
 * \retval SFX_ERR_TEST_MODE_5_BUILD_UPLINK:      Build uplink frame failed
 * \retval SFX_ERR_TEST_MODE_5_SEND_UPLINK:       Send uplink frame failed
 * \retval SFX_ERR_TEST_MODE_5_RF_STOP:           Error on MANUF_API_rf_stop
 *******************************************************************/
st_sfx_error_t ST_SIGFOX_API_test_mode(st_sfx_test_mode_t test_mode, uint8_t config);



/** @}*/

#endif /* ST_SFX_API_H_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
