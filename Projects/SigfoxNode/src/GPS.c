/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>

#include "GPS.h"

/* Private define ------------------------------------------------------------*/
#define WAKE_PIN GPIO_PIN_7
#define WAKE_PIN_BANK GPIOC

#define NRES_PIN GPIO_PIN_6
#define NRES_BANK GPIOB

#define ON_OFF_PIN GPIO_PIN_10
#define ON_OFF_BANK GPIOB

#define PPS_PIN GPIO_PIN_2
#define PPS_BANK GPIOD

#define NMEA_LEN 82

#define MAX_TRYS_GPS 5

#define N_PTF_SAMPLES 10

/* Private variables ---------------------------------------------------------*/
uint8_t sirfInitDataSource[] = { 0x80, 	// Message ID 128
		0x00, 0x39, 0xB4, 0x59, 	// ECEF X
		0x00, 0x0D, 0xBB, 0x54, 	// ECEF Y
		0x00, 0x4C, 0xE6, 0x5F, 	// ECEF Z
		0x00, 0x01, 0x24, 0xF8, 	// Clock Drift = 75000 Hz
		0x00, 0x00, 0x00, 0x00, 	// Time of week [sec.]
		0x07, 0xBB, 				// GPS Week Number
		0x0C, 						// N Channels = 12
		0x03 						// Reset configuration
};

/* Set TricklePower Parameters – Message ID 151  */
uint8_t sirfPTFModePayload[9] = { 0x97,	// Message ID (hex)
		0x00, 0x01, // PTF mode = ON
		0x03, 0xE8,	// Duty Cycle (%)
		0x00, 0x00, 0x00, 0xC8,	// ON-Time (200 - 900 msec)
};

/* Set Low Power Acquisition Parameters – Message ID 167 */
//uint8_t sirfPTFPeriodPayload[15] = { 0xA7, 	// Message ID (hex)
//		0x00, 0x04, 0x93, 0xE0,  // Max Off Time [ms] ==> 300000ms = 300s = 5min
//		0x00, 0x00, 0x75, 0x30,	// Max Search Time [ms] ==> 30000ms = 30s = 0.5min
//		0x00, 0x00, 0x01, 0x2C,	// PTF Period [sec] ==> 300s = 5min
//		0x00, 0x00,		// Enable ATP = OFF
//};

/* Set Low Power Acquisition Parameters – Message ID 167 */
uint8_t sirfPTFPeriodPayload[15] = { 0xA7, 	// Message ID (hex)
		0x00, 0x1B, 0x77, 0x40,  // Max Off Time [ms] ==> 1800000ms = 1800s = 30min
		0x00, 0x00, 0x75, 0x30,	// Max Search Time [ms] ==> 30000ms = 30s = 0.5min
		0x00, 0x00, 0x07, 0x08,	// PTF Period [sec] ==> 1800s = 30min
		0x00, 0x00,		// Enable ATP = OFF
};

/* DGPS Source – Message ID 133 */
uint8_t sirfEnableSBASPayload[7] = { 0x85, // Message ID (hex)
		0x01,	//DGPS source: 00 = None, 01 = SBAS, 02 = External RTCM data, 03 = Internal DGPS beacon receiver, 04 = User software
		0x00, 0x00, 0x00, 0x00, 	// Not used
		0x00, 	// Not used
};

/* Set SBAS Parameters - Message ID 170 */
uint8_t sirfSBASParametersPayload[6] = { 0xAA, 	// Message ID (hex)
		0x03, 	// SBAS Region: 00 = auto, 02 = WAAS, 03 = EGNOS, 04 = MSAS, 05 = GAGAN
		0x00,	// SBAS Mode: 00 = Testing (accept satellite connection in test mode), 01 = Integrity
		0x00, 	// Flag Bits
		0x03,	// Region
		0x88,	// PRN for region (EGNOS SES-5 5°East PRN = 136)
};

uint8_t sirfToNMEAPayload[24] = { 0x81,	// Message ID (hex)
		0x02, 	// Debug Msgs: 0 = ON, 1 = OFF, 2 = dont change
		0x01, 0x01, 0x00, 0x01,	// GGA ON, GGA CRC ON, GLL OFF, GLL CRC ON
		0x01, 0x01, 0x01, 0x01,	// GSA + GSV
		0x01, 0x01, 0x00, 0x01,	// RMC + VTG
		0x00, 0x01, 0x00, 0x01,	// MSS + EPE
		0x00, 0x01, // ZDA
		0x00, 0x00, // UNUSED
		0x25, 0x80,	// Bit rate = 9600 (don't care)
};

char nmeaSetSirfProt[] = "$PSRF100,0,9600,8,1,0*%02X\r\n";

char OKtoSend[] = "$PSRF150,1*3E\r\n";
uint8_t OKtoSendLen = sizeof(OKtoSend)/sizeof(char) - 1;

char GPStxOff[] = "$PSRF150,0*3F\r\n";
uint8_t GPStxOffLen = sizeof(GPStxOff)/sizeof(char) - 1;

bool SensorBoardPresent = true;

GPSData_t GPSDataBuffer;

/* Private function prototypes -----------------------------------------------*/
static char calcNMEACRC(const char *nmeaMsg, uint16_t len);
static uint16_t calcSIRFCRC(uint8_t *sirfPayload, uint16_t sirfLen);
static int checkForProtocol( void );

/* Exported functions ---------------------------------------------------------*/
void GPSPTFRequest(GPSData_t *GPSData) {
	uint8_t samples = 0;

	GPSData_t GPSBuf = {0};

	if(HAL_GPIO_ReadPin(WAKE_PIN_BANK, WAKE_PIN) != 1) {
		/* PTF request with ON/OFF pulse */
		HAL_GPIO_WritePin(ON_OFF_BANK, ON_OFF_PIN, GPIO_PIN_SET);
		SdkDelayMs(100);
		HAL_GPIO_WritePin(ON_OFF_BANK, ON_OFF_PIN, GPIO_PIN_RESET);
	}

	while( 1 ){

		GPSRead(&GPSBuf);
		if(GPSBuf.messageType == GPS_IDLE_MESSAGE) continue;

		if(GPSBuf.messageType == GPGGA_MESSAGE || GPSBuf.messageType == RMC_MESSAGE) {

			if(hasFix(GPSBuf)) {
				*GPSData = GPSBuf;
			}

			samples++;

			memset((void*)&GPSBuf, 0, sizeof(GPSBuf));

		} else if(GPSBuf.messageType == GPS_TX_OFF_MESSAGE) {
			printf("PTF period end! \n");
			break;
		}

		if(samples >= N_PTF_SAMPLES) {
			printf("GPSPTFRequest: Reached max samples\n");
			break;
		}
	}
}

void GPSRead(GPSData_t *gpsData){
	uint8_t nmeaMsg[NMEA_LEN];
	uint16_t len;

	if(SensorBoardPresent == false) {
		printf("GPSRead: SensorBoardPresent == false \n");
		gpsData->messageType = GPS_TX_OFF_MESSAGE;
		return;
	}

	if(HAL_GPIO_ReadPin(WAKE_PIN_BANK, WAKE_PIN) == false ) {
		//printf("GPSRead: GPS not available\n");
		gpsData->messageType = GPS_TX_OFF_MESSAGE;
		return;
	}

	GPS_READ_NMEA(nmeaMsg, NMEA_LEN, &len);
	if(len == 0) {
		//printf("GPSRead: Got no nmeaMesg from GPS\n");
		gpsData->messageType = GPS_IDLE_MESSAGE;
		return;
	}

	GPSParseNMEA((const char*)nmeaMsg, gpsData);

	if(gpsData->messageType == GPGGA_MESSAGE || gpsData->messageType == RMC_MESSAGE) {
		printf("%s \n", (const char *) nmeaMsg);
	}
}

void GPSSetup( void ) {
	char command[100] = {0};
	char crc;
	uint16_t len = 0;
	uint16_t sirfCrc = 0;

	printf("GPSSetup: Begin\n");

	if(GPSInitHW() < 0 ) {
		return;
	}

	/* NMEA switch to SIRF command */
	len = strlen((const char *) nmeaSetSirfProt);
	crc = calcNMEACRC(nmeaSetSirfProt, len);
	snprintf(command, 100, nmeaSetSirfProt, crc);
	len = strlen((const char *) command);
	GPS_SEND_NMEA((uint8_t *)command, len);

	/* Enable PTF Mode command */
	len = sizeof(sirfPTFModePayload) / sizeof(uint8_t);
	sirfCrc = calcSIRFCRC(sirfPTFModePayload, len);
	GPS_SEND_SIRF(sirfPTFModePayload, len, sirfCrc, true);

	/* Set PTF period command */
	len = sizeof(sirfPTFPeriodPayload) / sizeof(uint8_t);
	sirfCrc = calcSIRFCRC(sirfPTFPeriodPayload, len);
	GPS_SEND_SIRF(sirfPTFPeriodPayload, len, sirfCrc, true);

	/* SBAS parameters */
	len = sizeof(sirfSBASParametersPayload) / sizeof(uint8_t);
	sirfCrc = calcSIRFCRC(sirfSBASParametersPayload, len);
	GPS_SEND_SIRF(sirfSBASParametersPayload, len, sirfCrc, true);

	/* Activate SBAS */
	len = sizeof(sirfEnableSBASPayload) / sizeof(uint8_t);
	sirfCrc = calcSIRFCRC(sirfEnableSBASPayload, len);
	GPS_SEND_SIRF(sirfEnableSBASPayload, len, sirfCrc, true);

	/* Switch back to NMEA protocol command */
	len = sizeof(sirfToNMEAPayload) / sizeof(uint8_t);
	sirfCrc = calcSIRFCRC(sirfToNMEAPayload, len);
	GPS_SEND_SIRF(sirfToNMEAPayload, len, sirfCrc, false);

}

int GPSInitHW( void ) {
	GPIO_InitTypeDef gpsGPIO = {0};
	uint8_t trys = 0;
	int ret = 0;
	uint8_t answer[GPS_TX_FIFO_SIZE] = {0};

	/*
	 * NRES -> PA9 OUTPUT
	 * WAKE -> PC7 INPUT
	 * ON/OFF -> PB10 OUTPUT
	 * PPS(RTC value) -> PD2 INPUT
	 *
	 */

	gpsGPIO.Pin = WAKE_PIN;
	gpsGPIO.Mode = GPIO_MODE_INPUT;
	gpsGPIO.Pull = GPIO_NOPULL;
	gpsGPIO.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(WAKE_PIN_BANK, &gpsGPIO);

	gpsGPIO.Pin = PPS_PIN;
	HAL_GPIO_Init(PPS_BANK, &gpsGPIO);

	gpsGPIO.Pin = ON_OFF_PIN;
	gpsGPIO.Mode = GPIO_MODE_OUTPUT_PP;
	gpsGPIO.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ON_OFF_BANK, &gpsGPIO);

	gpsGPIO.Pin = NRES_PIN;
	gpsGPIO.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NRES_BANK, &gpsGPIO);

	/* Set NRES low for 100ms */
	HAL_GPIO_WritePin(NRES_BANK, NRES_PIN, GPIO_PIN_RESET);
	SdkDelayMs(100);

	/* Leave NRES open, so GPS Module can drive it again */
	gpsGPIO.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(NRES_BANK, &gpsGPIO);

	/* Initialize SPI2 */
	GPS_SPI_Init( );

	printf("GPSInitHW: Checking GPS module WAKE PIN\n");
	/* Send ON/OFF pulses with ON_OFF_PIN until WAKE_PIN indicates
	 * that the GPS Module is in full power state */
	while(!HAL_GPIO_ReadPin(WAKE_PIN_BANK, WAKE_PIN)) {
		HAL_GPIO_WritePin(ON_OFF_BANK, ON_OFF_PIN, GPIO_PIN_SET);
		SdkDelayMs(100);
		HAL_GPIO_WritePin(ON_OFF_BANK, ON_OFF_PIN, GPIO_PIN_RESET);

		trys++;
		printf("GPSInitHW: Retry %d\n", trys);
		if(trys >= MAX_TRYS_GPS){
			printf("GPSInitHW: No wake Signal from GPS Module after %u trys\n", trys);
			SensorBoardPresent = false;
			return -1;
		}

		/* Datasheet: ON_OFF pulses with less than 1s intervals are not recommended. */
		SdkDelayMs(1000);
	}

	trys = 0;

	ret = checkForProtocol();
	if(ret == 1) {
		/* GPS protocol is SiRF Switch back to NMEA */
		uint16_t len = sizeof(sirfToNMEAPayload) / sizeof(uint8_t);
		uint16_t sirfCrc = calcSIRFCRC(sirfToNMEAPayload, len);
		GPS_SEND_SIRF(sirfToNMEAPayload, len, sirfCrc, false);
	}

	printf("GPSInitHW: Waiting for OktoSend msg\n");
	while(1) {
		GPS_READ(answer, GPS_TX_FIFO_SIZE);

		if(strncmp((const char *)answer, OKtoSend, OKtoSendLen) == 0) {
			break;
		}

		trys++;

		printf("GPSInitHW: Retry %d\n", trys);
		if(trys >= MAX_TRYS_GPS){
			printf("GPSInitHW: Did not receive OK to send\n");
			return -1;
		}

		GPSReset();

		SdkDelayMs(500);
	}

	printf("GPSInitHW: GPS init success\n");

	return 0;
}

void GPSComInit( void ) {
	GPIO_InitTypeDef gpsGPIO = {0};

	/*
	 * NRES -> PB6 OUTPUT
	 * WAKE -> PC7 INPUT
	 * ON/OFF -> PB10 OUTPUT
	 * PPS(RTC value) -> PD2 INPUT
	 *
	 */

	gpsGPIO.Pin = WAKE_PIN;
	gpsGPIO.Mode = GPIO_MODE_INPUT;
	gpsGPIO.Pull = GPIO_NOPULL;
	gpsGPIO.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(WAKE_PIN_BANK, &gpsGPIO);

	gpsGPIO.Pin = PPS_PIN;
	HAL_GPIO_Init(PPS_BANK, &gpsGPIO);

	gpsGPIO.Pin = ON_OFF_PIN;
	gpsGPIO.Mode = GPIO_MODE_OUTPUT_PP;
	gpsGPIO.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ON_OFF_BANK, &gpsGPIO);

	gpsGPIO.Pin = NRES_PIN;
	gpsGPIO.Pull = GPIO_NOPULL;
	gpsGPIO.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(NRES_BANK, &gpsGPIO);

	/* Initialize SPI2 */
	GPS_SPI_Init( );
}

void GPSReset( void ) {
	GPIO_InitTypeDef gpsGPIO = {0};
	uint8_t trys = 0;

	gpsGPIO.Pin = NRES_PIN;
	gpsGPIO.Mode = GPIO_MODE_OUTPUT_PP;
	gpsGPIO.Pull = GPIO_NOPULL;
	gpsGPIO.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(NRES_BANK, &gpsGPIO);

	/* Set NRES low for 100ms */
	HAL_GPIO_WritePin(NRES_BANK, NRES_PIN, GPIO_PIN_RESET);
	SdkDelayMs(100);

	/* Leave NRES open, so GPS Module can drive it again */
	gpsGPIO.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(NRES_BANK, &gpsGPIO);

	printf("resetGPS: Checking GPS module WAKE PIN\n");
	/* Send ON/OFF pulses with ON_OFF_PIN until WAKE_PIN indicates
	 * that the GPS Module is in full power state */
	while(!HAL_GPIO_ReadPin(WAKE_PIN_BANK, WAKE_PIN)) {
		HAL_GPIO_WritePin(ON_OFF_BANK, ON_OFF_PIN, GPIO_PIN_SET);
		SdkDelayMs(100);
		HAL_GPIO_WritePin(ON_OFF_BANK, ON_OFF_PIN, GPIO_PIN_RESET);

		trys++;
		printf("resetGPS: Retry %d\n", trys);
		if(trys >= MAX_TRYS_GPS){
			printf("resetGPS: No wake Signal from GPS Module\n");
			return;
		}

		/* Datasheet: ON_OFF pulses with less than 1s intervals are not recommended. */
		SdkDelayMs(1000);
	}

	printf("resetGPS: Done\n");
}

bool hasFix(GPSData_t data) {

	if(data.messageType == RMC_MESSAGE) {
		if(data.valid) return true;

		return false;

	}else if(data.messageType == GPGGA_MESSAGE) {
		if(data.fixQuality > 0) return true;

		return false;
	}else {
		return false;
	}
}

static int checkForProtocol( void ) {
	uint8_t buf[GPS_TX_FIFO_SIZE];
	uint16_t len = 0;

	GPS_READ_NMEA(buf, GPS_TX_FIFO_SIZE, &len);
	if(len != 0) {
		printf("checkForProtocol: Got NMEA Msg \n");
		return 0;
	}

	GPS_READ_SIRF(buf, GPS_TX_FIFO_SIZE, &len);
	if(len != 0) {
		printf("checkForProtocol: Got SiRF Msg \n");
		return 1;
	}

	return -1;
}

static char calcNMEACRC(const char *nmeaMsg, uint16_t len) {
	uint16_t i;
	char crc = 0;

	for(i = 1; i < len; i ++) {
		/* 0x2A = * = NMEA CRC start*/
		if(nmeaMsg[i] == 0x2A) break;
		crc = (char)(crc ^ nmeaMsg[i]);
	}

	return crc;
}

static uint16_t calcSIRFCRC(uint8_t *sirfPayload, uint16_t sirfLen) {
	uint16_t crc = 0;
	uint16_t i = 0;

	for(i = 0; i < sirfLen; i++) {
		crc += sirfPayload[i];
	}

	crc &= 0x7FFF;

	return crc;
}

void GPSParseNMEA(const char *message, GPSData_t *gpsData) {

	if(strncmp(OKtoSend, message, OKtoSendLen) == 0) {
		printf("GPSParseNMEA: received OKtoSend! \n");
		gpsData->messageType = OK_TO_SEND_MESSAGE;
		return;
	}

	if(strncmp(GPStxOff, message, GPStxOffLen) == 0) {
		printf("GPSParseNMEA: received GPStxOff! \n");
		gpsData->messageType = GPS_TX_OFF_MESSAGE;
		return;
	}

	switch(minmea_sentence_id(message, false)) {
	case MINMEA_SENTENCE_RMC: {
		gpsData->messageType = RMC_MESSAGE;

		struct minmea_sentence_rmc rmcData;
		if(minmea_parse_rmc(&rmcData, message)) {
			if(rmcData.time.hours > 0) {
				gpsData->time = rmcData.time.hours;
				gpsData->time = gpsData->time * 100 + rmcData.time.minutes;
				gpsData->time = gpsData->time * 100 + rmcData.time.seconds;

				if(rmcData.time.microseconds > 0) {
					//gpsData->time = gpsData->time * 1000 + (rmcData.time.microseconds / 1000);
				}
			}else {
				gpsData->time = 0;
			}

			if(rmcData.date.year > 0) {
				gpsData->date = rmcData.date.day;
				gpsData->date = gpsData->date * 100 + rmcData.date.month;
				gpsData->date = gpsData->date * 100 + rmcData.date.year;
			}else {
				gpsData->date = 0;
			}

			gpsData->valid = rmcData.valid;
			gpsData->latitude = minmea_rescale(&rmcData.latitude, rmcData.latitude.scale);
			gpsData->longitude = minmea_rescale(&rmcData.longitude, rmcData.longitude.scale);
			gpsData->speed = rmcData.speed.value;
			gpsData->speedScale = rmcData.speed.scale;
		}
	} break;

	case MINMEA_SENTENCE_GGA: {
		gpsData->messageType = GPGGA_MESSAGE;

		struct minmea_sentence_gga ggaData;
		if (minmea_parse_gga(&ggaData, message)) {

			if(ggaData.time.hours > 0) {
				gpsData->time = ggaData.time.hours;
				gpsData->time = gpsData->time * 100 + ggaData.time.minutes;
				gpsData->time = gpsData->time * 100 + ggaData.time.seconds;
				if(ggaData.time.microseconds) {
					//gpsData->time = gpsData->time * 1000 + (ggaData.time.microseconds / 1000);
				}
			}else {
				gpsData->time = 0;
			}

			gpsData->latitude = minmea_rescale(&ggaData.latitude, ggaData.latitude.scale);
			gpsData->longitude = minmea_rescale(&ggaData.longitude, ggaData.longitude.scale);
			gpsData->fixQuality = ggaData.fix_quality;
			gpsData->nSatellites = ggaData.satellites_tracked;
			gpsData->hdop = ggaData.hdop.value;
			gpsData->hdopScale = ggaData.hdop.scale;

			gpsData->altitudeSee = ggaData.altitude.value;
			gpsData->altitude = ggaData.height.value;
		}
	} break;

	case MINMEA_SENTENCE_GSA: {
		struct minmea_sentence_gsa gsaData;

		gpsData->messageType = GPGSA_MESSAGE;

		if (minmea_parse_gsa(&gsaData, message)) {
			printf("Satelites active: ");

			for(uint8_t i = 0; i < 12; i++) {
				printf("%d ", gsaData.sats[i]);
			}

			printf("\n");

		}
	} break;

	case MINMEA_SENTENCE_GLL: {

	} break;

	case MINMEA_SENTENCE_GST: {

	} break;

	case MINMEA_SENTENCE_VTG: {

	} break;

	case MINMEA_SENTENCE_GSV: {
		struct minmea_sentence_gsv gsvData;

		gpsData->messageType = GPGSV_MESSAGE;
		if (minmea_parse_gsv(&gsvData, message)) {
			printf("Satelites in view message %d of %d - nr [snr]: ", gsvData.msg_nr, gsvData.total_msgs);

			for(uint8_t i = 0; i < 4; i ++) {
				printf("%d [%d]; ", gsvData.sats[i].nr, gsvData.sats[i].snr);
			}

			printf("\n");
		}
	} break;

	case MINMEA_UNKNOWN: {
		gpsData->messageType = UNKNOWN_MESSAGE;
	} break;

	default: {
		gpsData->messageType = UNKNOWN_MESSAGE;
	} break;
	}
}
