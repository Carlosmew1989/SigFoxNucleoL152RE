/*
 * hw_gps_spi.h
 *
 *  Created on: 22.08.2017
 *      Author: wisbar
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_GPS_SPI_H_
#define HW_GPS_SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <string.h>
#include <stdbool.h>
#include "SDK_EVAL_Config.h"
#include "hw_spi.h"

/* Exported constants --------------------------------------------------------*/
#define GPS_TX_FIFO_SIZE 1024

 /* Exported functions ------------------------------------------------------- */

void GPS_SPI_Init( void );

void GPS_SPI2_IoInit( void );

void GPS_SPI_DeInit( void );

void GPS_READ( uint8_t *rxbuff, uint16_t buffSize);

void GPS_SET_SLEEP_MODE( void );

void GPS_READ_NMEA( uint8_t *rxData, uint16_t buffSize, uint16_t *nmeaLen);

void GPS_READ_SIRF( uint8_t *rxData, uint16_t buffSize, uint16_t *sirfLen );

void GPS_SEND_NMEA( uint8_t *nmeaMsg, uint16_t len );

void GPS_SEND_SIRF(const uint8_t *sirfPayload, uint16_t payloadLen, uint16_t crc, bool checkAck);

#ifdef __cplusplus
}
#endif

#endif /* HW_GPS_SPI_H_ */
