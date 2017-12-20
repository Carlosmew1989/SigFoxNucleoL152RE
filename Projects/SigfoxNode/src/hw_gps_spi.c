/*
 * hw_gps_spi.c
 *
 *  Created on: 22.08.2017
 *      Author: wisbar
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_gps_spi.h"

/* Private define ------------------------------------------------------------*/
#define GPS_NCS_PIN GPIO_PIN_1
#define GPS_NCS_BANK GPIOB

#define SIRF_OVERHEAD 8// 2byte start sequence + 2 byte payload length + 2 byte CRC + 2byte end sequence

void GPS_SPI_IoDeInit( void );

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef hspi_gps;

uint8_t nmeaSleep[] = "$PSRF117,16*0B\r\n";

/* Exported functions ---------------------------------------------------------*/
void GPS_SPI_Init( void ) {

	/* SPI Mode 1: CPOL = 0, CPHA = 1 */
	hspi_gps.Instance = SPI2;
	hspi_gps.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi_gps.Init.Direction = SPI_DIRECTION_2LINES;
	hspi_gps.Init.Mode = SPI_MODE_MASTER;
	hspi_gps.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi_gps.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi_gps.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi_gps.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi_gps.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi_gps.Init.NSS = SPI_NSS_SOFT;
	hspi_gps.Init.TIMode = SPI_TIMODE_DISABLE;

	SPI2_CLK_ENABLE();

	if (HAL_SPI_Init(&hspi_gps) != HAL_OK)
	{
		printf("FATAL ERROR GPS_SPI_Init\n");
		while(1) {}
	}

	/* Initialize SPI2 GPIO Pins */
	GPS_SPI2_IoInit();
}

void GPS_SPI2_IoInit( void )
{
	GPIO_InitTypeDef initStruct={0};

	/*
	 * SPI2 AF5 config:
	 * PB13 -> SPI2 CLOCK
	 * PB14 -> SPI2 MISO
	 * PB15 -> SPI2 MOSI
	 * PA5  -> NChipSelect GPS modul
	 */

	initStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	initStruct.Mode = GPIO_MODE_AF_PP;
	initStruct.Pull = GPIO_NOPULL;
	initStruct.Speed = GPIO_SPEED_HIGH;
	initStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &initStruct);

	initStruct.Pin = GPS_NCS_PIN;
	initStruct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(GPS_NCS_BANK, &initStruct);

	initStruct.Pin = BMC_NCS_PIN;
	HAL_GPIO_Init(BMC_NCS_BANK, &initStruct);

	initStruct.Pin = BME_NCS_PIN;
	HAL_GPIO_Init(BME_NCS_BANK, &initStruct);

	/* Set NCS high to disable slave communication */
	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (BMC_NCS_BANK, BMC_NCS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (BME_NCS_BANK, BME_NCS_PIN, GPIO_PIN_SET);

}

void GPS_SPI_DeInit( void ) {
	HAL_SPI_DeInit( &hspi_gps);

	/*##-1- Reset peripherals ####*/
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  /*##-2- Configure the SPI GPIOs */
  GPS_SPI_IoDeInit( );
}

void GPS_SPI_IoDeInit( void ) {
  GPIO_InitTypeDef initStruct={0};

  initStruct.Pin = GPS_NCS_PIN;
  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init ( GPS_NCS_BANK, &initStruct );
  HAL_GPIO_WritePin( GPS_NCS_BANK, GPS_NCS_PIN , GPIO_PIN_SET );

}

void GPS_READ( uint8_t *rxbuff, uint16_t buffSize ) {
	uint16_t i, numSymbols = 0;
	uint8_t c;

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_RESET);

	for(i = 0; i < buffSize; i++) {
	  HAL_SPI_Receive(&hspi_gps, &c, 1, HAL_MAX_DELAY);

	  /* Ignore FIFO idle pattern */
	  if(c == 0xA7 || c == 0xB4) continue;

	  rxbuff[numSymbols++] = c;
	}

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);

	rxbuff[numSymbols] = '\0';

	return;
}

void GPS_SET_SLEEP_MODE( void ) {
	uint16_t i;
	uint8_t c;
	uint8_t len = strlen((const char *) nmeaSleep);

	printf("Sending sleep command: %s\n", nmeaSleep);

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi_gps, nmeaSleep, len, HAL_MAX_DELAY);

	printf("Buffer readout: \n");
	for(i = 0; i < GPS_TX_FIFO_SIZE; i++) {
		HAL_SPI_Receive(&hspi_gps, &c, 1, HAL_MAX_DELAY);
		printf("%c",c);
	}
	printf("\n");

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);
}

void GPS_READ_NMEA( uint8_t *rxData, uint16_t buffSize, uint16_t *nmeaLen){
  uint8_t c, numSymbols = 0;
  bool nmeaStart = false;
  uint16_t i = 0;

  /* read max. 1024 Bytes */
  uint16_t bytesToRead = buffSize < 512U ? 2*buffSize : buffSize;

  HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_RESET);

  //printf("GPS_READ_NMEA: SPI data: ");

  for(i = 0; i < bytesToRead; i++) {
	  HAL_SPI_Receive(&hspi_gps, &c, 1, HAL_MAX_DELAY);

	  /* Ignore FIFO idle pattern */
	  if(c == 0xA7 || c == 0xB4) continue;

	  //printf("%c", c);

	  if(!nmeaStart) {
		  // Look for start of NMEA message
		  if(c == 0x24) { 		// NMEA start symbol: $
			  rxData[numSymbols++] = c;
			  nmeaStart = true;
		  }

	  }else {
		  // Look for end of NMEA message
		  if(c == 0x0A) {   // NMEA messages always end with CR(0x0D) + LF(0x0A)
			  if(rxData[numSymbols-1] == 0x0D) {
				  rxData[numSymbols++] = c;
				  rxData[numSymbols++] = '\0';
				  break;
			  }
		  }

		  if(numSymbols < buffSize) {
			  rxData[numSymbols++] = c;
		  }else {
			  rxData[numSymbols-1] = '\0';
			  break;
		  }
	  }
  }

  HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);

  //printf("\n");

  if(!nmeaStart) {
	  rxData[0] = '\0';
	  *nmeaLen = 0;
  }else {
	  *nmeaLen = numSymbols;
  }

  return;
}

void GPS_SEND_NMEA( uint8_t *nmeaMsg, uint16_t len ) {
	uint16_t i = 0;

	printf("Sending command: \n");
	for(i = 0; i < len; i++) {
		printf("%c", nmeaMsg[i]);
	}

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi_gps, nmeaMsg, len, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);

}

void GPS_SEND_SIRF(const uint8_t *sirfPayload, uint16_t payloadLen, uint16_t crc, bool checkAck) {
	uint16_t i = 0, answerLen = 0, pos = 0;
	uint8_t sirfMsg[payloadLen + SIRF_OVERHEAD];
	uint8_t buf[GPS_TX_FIFO_SIZE];
	bool ackReceived = false;

	sirfMsg[pos++] = 0xA0;
	sirfMsg[pos++] = 0xA2;
	sirfMsg[pos++] = (payloadLen >> 8) & 0xFF;
	sirfMsg[pos++] = payloadLen & 0xFF;

	memcpy(sirfMsg + pos, sirfPayload, payloadLen);

	pos += payloadLen;

	sirfMsg[pos++] = (crc >> 8) & 0xFF;
	sirfMsg[pos++] = crc & 0xFF;
	sirfMsg[pos++] = 0xB0;
	sirfMsg[pos++] = 0xB3;

	printf("GPS_SEND_SIRF: sending command\n");
	for(i = 0; i < pos; i++) {
		printf("%02X ", sirfMsg[i]);
	}
	printf("\n");

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi_gps, sirfMsg, pos, HAL_MAX_DELAY);

	if(checkAck){
		for(i = 0; i < 4; i++) {
			GPS_READ_SIRF(buf, GPS_TX_FIFO_SIZE, &answerLen);

			if(answerLen != 0) {
				/* SiRF ACK message ID */
				if(buf[4] == 0x0B) {
					/* Check if ACK is for current message */
					if(buf[5] == sirfPayload[0]) {
						printf("GPS_SEND_SIRF: Received ACK for command: %02X \n", buf[5]);
						ackReceived = true;
						break;
					}
				}

				/* SiRF negative ACK message ID */
				if(buf[4] == 0x0C) {
					/* Check if negative ACK is for current message */
					if(buf[5] == sirfPayload[0]) {
						printf("GPS_SEND_SIRF: Received NEGATIVE ACK for command: %02X \n", buf[5]);
						ackReceived = false;
						break;
					}
				}
			}
		}

		if(!ackReceived) {
			printf("GPS_SEND_SIRF: No ACK received for command %02X!\n", sirfPayload[0]);
		}
	}

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);
}

void GPS_READ_SIRF( uint8_t *rxData, uint16_t buffSize, uint16_t *sirfLen ) {
	bool sirfStartA2 = false, sirfStartA0 = false;
	uint8_t c;
	uint16_t i = 0, numSymbols = 0;

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_RESET);

	//printf("SPI SiRF data:\n");

	for(i = 0; i < buffSize; i++) {
		HAL_SPI_Receive(&hspi_gps, &c, 1, HAL_MAX_DELAY);
		//printf("%02X ", c);

		/* SiRF commands start with A0 A2 */
		if(!sirfStartA2) {

			if(!sirfStartA0) {
				if(c == 0xA0){
					rxData[numSymbols++] = c;
					sirfStartA0 = true;
				}
			}else {
				if(c == 0xA2) {
					rxData[numSymbols++] = c;
					sirfStartA2 = true;
				}
			}

		}else {

			if(numSymbols < buffSize) {
				rxData[numSymbols++] = c;
			}else {
				printf("HW_SPI_GPS_SIFR_InOut: SiRF msg does not fit in buffSize %u \n", buffSize);
				break;
			}

			/* SiRF commands end with B0 B3 */
			if(c == 0xB3) {
				if(rxData[numSymbols-1] == 0xB0) {
					break;
				}
			}
		}
	}

	HAL_GPIO_WritePin(GPS_NCS_BANK, GPS_NCS_PIN, GPIO_PIN_SET);

	//printf(" SiRF END\n\n");

	*sirfLen = sirfStartA2 ? numSymbols : 0;

	return;
}
