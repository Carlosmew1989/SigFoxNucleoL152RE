/**
* @file    sigfox_retriever.h
* @author  LowPower RF BU - AMG
* @version 1.1.0
* @date    November 9, 2016
* @brief   This is used to retrieve the SigFox data as ID, PAC and AES-KEY.
*          The AES-KEY is a private variable and is not returned to the user.
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
* <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
*/

#include <stdint.h>

/* Retriever error codes definition */
typedef uint8_t      retr_error_t;

/*!
 * \defgroup ST_SIGFOX_RETRIEVER
 *
 *  @{
 */

/*!
 * \defgroup RETRIEVER_ERR_CODES Return Error codes definition for the ST_SIGFOX_RETRIEVER
 *
 *  @{
 */
#define RETR_OK         0     /* no error */
#define RETR_ERR        1     /* error */

/** @}*/

/*!******************************************************************
 * \fn retr_error_t enc_utils_retrieve_data(uint32_t *id, uint8_t *pac, uint8_t *rcz)
 * \brief Retrieve the ID, PAC and RCZ number of the board and returns it to the caller.
 *        The ID should be used when opening the library. The PAC is used to register the node on the backend.
 * \param[in] id_ptr: pointer to the 32bits word variable where the ID of the board must be stored.
 * \param[in] pac_ptr: pointer to the 8bytes array where the PAC of the board must be stored.
 * \param[in] rcz_ptr: pointer to the byte where the RCZ number of this board must be stored.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_retrieve_data(uint32_t *id, uint8_t *pac, uint8_t *rcz);


/*!******************************************************************
 * \fn retr_error_t enc_utils_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint16_t data_len)
 * \brief Perform the AES128-CBC encryption using the AES KEY associated to the board.
 * \param[in] uint8_t* encrypted_data: pointer to the destination buffer where the encrypted data must be stored.
 * \param[in] uint8_t* data_to_encrypt: pointer to the source buffer where the data to encrypt are stored.
 * \param[in] uint16_t data_len: length of the data buffer to encrypt.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint16_t data_len);

/*!******************************************************************
 * \fn retr_error_t enc_utils_set_public_key(uint8_t en)
 * \brief Switch the encryption key to the public key: <i>0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF</i>.
   * \param[in] uint8_t en: if 1 switch to the public key, else restore the one associated with the board.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_set_public_key(uint8_t en);

/** @}*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
