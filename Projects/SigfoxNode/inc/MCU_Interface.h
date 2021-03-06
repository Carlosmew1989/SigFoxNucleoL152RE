/**
 * @file    MCU_Interface.h
 * @author  LowPower RF BU - AMG
 * @version 1.1.0
 * @date    July 1, 2016
 * @brief   Header file for low level S2LP SPI driver.
 * @details
 *
 * This header file constitutes an interface to the SPI driver used to
 * communicate with S2LP.
 * It exports some function prototypes to write/read registers and FIFOs
 * and to send command strobes.
 * Since the S2LP libraries are totally platform independent, the implementation
 * of these functions are not provided here. The user have to implement these functions
 * taking care to keep the exported prototypes.
 *
 * These functions are:
 *
 * <ul>
 * <li>S2LPSpiInit</i>
 * <li>S2LPSpiWriteRegisters</i>
 * <li>S2LPSpiReadRegisters</i>
 * <li>S2LPSpiCommandStrobes</i>
 * <li>S2LPSpiWriteLinearFifo</i>
 * <li>S2LPSpiReadLinearFifo</i>
 * </ul>
 *
 * @note An example of SPI driver implementation is available in the <i>Sdk_Eval</i> library.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_INTERFACE_H
#define __MCU_INTERFACE_H


/* Includes ------------------------------------------------------------------*/

#include "st_lowlevel.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup S2LP_Libraries
 * @{
 */

/** @defgroup S2LP_SPI_Driver         SPI Driver
 * @brief Header file for low level S2LP SPI driver.
 * @details See the file <i>@ref MCU_Interface.h</i> for more details.
 * @{
 */


/** @defgroup SPI_Exported_Types        SPI Exported Types
 * @{
 */
  /**
 * @brief  S2LP Functional state. Used to enable or disable a specific option.
 */
typedef enum {
  S_DISABLE = 0,
  S_ENABLE = !S_DISABLE
} SFunctionalState;


/**
 * @brief  S2LP Flag status. Used to control the state of a flag.
 */
typedef enum {
  S_RESET = 0,
  S_SET = !S_RESET
} SFlagStatus;


/**
 * @brief  boolean type enumeration.
 */
typedef enum {
  S_FALSE = 0,
  S_TRUE  = !S_FALSE
} SBool;


/**
 * @brief  S2LP States enumeration.
 */
typedef enum {
  MC_STATE_READY             =0x00,  /*!< READY */
  MC_STATE_SLEEP_NOFIFO      =0x01,  /*!< SLEEP NO FIFO RETENTION */
  MC_STATE_STANDBY           =0x02,  /*!< STANDBY */
  MC_STATE_SLEEP             =0x03,  /*!< SLEEP */
  MC_STATE_LOCKON            =0x0C,  /*!< LOCKON */
  MC_STATE_RX                =0x30,  /*!< RX */
  MC_STATE_LOCK_ST           =0x14,  /*!< LOCK_ST */
  MC_STATE_TX                =0x5C,  /*!< TX */
  MC_STATE_SYNTH_SETUP       =0x50   /*!< SYNTH_SETUP */
} S2LPState;


/**
 * @brief S2LP Status. This definition represents the single field of the S2LP
 *        status returned on each SPI transaction, equal also to the MC_STATE registers.
 *        This field-oriented structure allows user to address in simple way the single
 *        field of the S2LP status.
 *        The user shall define a variable of S2LPStatus type to access on S2LP status fields.
 * @note  The fields order in the structure depends on used endianness (little or big
 *        endian). The actual definition is valid ONLY for LITTLE ENDIAN mode. Be sure to
 *        change opportunely the fields order when use a different endianness.
 */
typedef struct {
  uint8_t XO_ON:1;           /*!< XO is operating state */
  S2LPState MC_STATE: 7;     /*!< The state of the Main Controller of S2LP @ref S2LPState */
  uint8_t ERROR_LOCK: 1;     /*!< RCO calibration error */
  uint8_t RX_FIFO_EMPTY: 1;  /*!< RX FIFO is empty */
  uint8_t TX_FIFO_FULL: 1;   /*!< TX FIFO is full */
  uint8_t ANT_SELECT: 1;     /*!< Currently selected antenna */
  uint8_t RCCAL_OK: 1;       /*!< RCO successfully terminated */
  uint8_t : 3;               /*!< This 3 bits field are reserved and equal to 2 */
}S2LPStatus;

typedef S2LPStatus StatusBytes;

/**
 * @}
 */



/** @defgroup SPI_Exported_Functions    SPI Exported Functions
 * @{
 */


void SdkEvalSpiInit(void);
void SdkEvalSpiDeinit(void);
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

void SdkEvalEnterShutdown(void);
void SdkEvalExitShutdown(void);
SFlagStatus SdkEvalCheckShutdown(void);

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */



#ifdef __cplusplus
}
#endif

#endif  /* __MCU_INTERFACE_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
