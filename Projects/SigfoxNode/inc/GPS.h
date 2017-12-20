/*
 * GPS.h
 *
 *  Created on: 18.08.2017
 *      Author: wisbar
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_H__
#define __GPS_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "SDK_EVAL_Config.h"
#include "hw_gps_spi.h"
//#include "hw_spi.h"
#include "minmea.h"

 /* Exported types ------------------------------------------------------------*/
typedef enum {
	 UNKNOWN_MESSAGE = 0,
	 RMC_MESSAGE = 1,
	 GPGGA_MESSAGE = 2,
	 GPGSA_MESSAGE = 3,
	 GPGSV_MESSAGE = 4,
	 OK_TO_SEND_MESSAGE = 5,
	 GPS_TX_OFF_MESSAGE = 6,
	 GPS_IDLE_MESSAGE = 7
 } NMEAMessageType;

 typedef struct {
	 NMEAMessageType messageType;
	 uint32_t time;
	 uint32_t date;
	 bool valid;	//A = active, V = void
	 int32_t latitude;
	 int32_t longitude;
	 int16_t speed;
	 uint16_t speedScale;
	 int16_t altitude;
	 int16_t altitudeSee;
	 uint8_t fixQuality;
	 uint8_t nSatellites;
	 uint8_t hdop;
	 uint16_t hdopScale;
 }GPSData_t;

 /* Exported constants --------------------------------------------------------*/
 /* External variables --------------------------------------------------------*/

 /* Exported functions ------------------------------------------------------- */
void GPSSetup( void );

int GPSInitHW( void );
void GPSComInit( void );

void GPSReset( void );

void GPSRead(GPSData_t *gpsData);

void GPSRead1(GPSData_t *gpsData);

void GPSParseNMEA(const char *message, GPSData_t *gpsData);

void GPSGetData(GPSData_t *GPSData);

void GPSPTFRequest(GPSData_t *GPSData);

bool hasFix(GPSData_t data);

void setGPSTimerEnable(bool val);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H__ */
