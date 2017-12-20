/**
 * @file    main.c
 * @author  AMG - RF Application team
 * @version 1.1.1
 * @date    April 1, 2017
 * @brief   SigFox Push Button Demo Application
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

/**
 * @file main.c
 * @brief This is a ST-SigFox demo that shows how to use the sigfox protocol to 
 *         a message to the base stations each time the push button is pressed.
 *         The data sent is a number representing the number of times the button 
 *              has been pressed from the boot.

 * \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt> ...\\Projects\\Projects_Cube\\SigFox_Applications\\SigFox_PushButton_Project\\MDK-ARM\\SigFox_PushButton_Project.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# Select Project->Download to download the related binary image.

 * \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> ...\\Projects\\Projects_Cube\\SigFox_Applications\\SigFox_PushButton_Project\\EWARM\\SigFox_PushButton_Project.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# Select Project->Download and Debug to download the related binary image.

 * \subsection Project_configurations Project configurations
- \c NUCLEO_L1_ETSI - Configuration to be used for RCZ1 on the NUCLEO-L152RE
- \c NUCLEO_L1_FCC - Configuration to be used for RCZ2 and RCZ4 on the NUCLEO-L152RE
- \c NUCLEO_L0_ETSI - Configuration to be used for RCZ1 on the NUCLEO-L053R8
- \c NUCLEO_L0_FCC - Configuration to be used for RCZ2 and RCZ4 on the NUCLEO-L053R8
- \c NUCLEO_F0_ETSI - Configuration to be used for RCZ1 on the NUCLEO-F072RB
- \c NUCLEO_F0_FCC - Configuration to be used for RCZ2 and RCZ4 on the NUCLEO-F072RB
- \c NUCLEO_F4_ETSI - Configuration to be used for RCZ1 on the NUCLEO-F401RE
- \c NUCLEO_F4_FCC - Configuration to be used for RCZ2 and RCZ4 on the NUCLEO-F401RE

 * \section Board_supported Boards supported
- \c STEVAL-FKI868V1 (for RCZ1)
- \c STEVAL-FKI915V1 (for RCZ2 and RCZ4)

 **/

/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "stddef.h"
#include "st_sigfox_api.h"
#include "sigfox_retriever.h"
#include "S2LP_SDK_Util.h"
#include "MCU_Interface.h"
#include "st_lowlevel_utils.h"

#include "GPS.h"
#include "hw_spi.h"

#define TX_DUTY_CYCLE 300 /* [Sec.] */

#define GPS_READ_CYCLE 1 /* [Sec.] */

#define SPI_MODE_GPS 0
#define SPI_MODE_BMx 1

uint8_t prepareSigfoxPayload(uint8_t *dataBuf, uint32_t lat, uint32_t lon, uint8_t speed, uint32_t time, uint16_t altitude);

uint32_t sampleStartPressure(uint8_t numSamples);

void switchSPIMode(uint8_t SPIMode);

/* a flag to understand if the button has been pressed */
static volatile uint8_t but_pressed = 0U;

static volatile uint8_t sendPacketTimer = 0U;

static volatile uint8_t readGPS = 0U;

uint16_t intCounter = 0U;

TIM_HandleTypeDef GPSTimerHandle;

GPSData_t GPSLastFix = {0};

/*
 * z = (R * T / g * M) * log(P0/P)
 * preFactor = R / g * M
 * */
const float preFactor = 29.273542;

void TIM7_IRQHandler( void ) {
	/* check if UIF Flag is set */
	if(GPSTimerHandle.Instance->SR & TIM_SR_UIF) {

		/* clear UIF flag */
		GPSTimerHandle.Instance->SR &= ~TIM_SR_UIF;

		intCounter += GPS_READ_CYCLE;

		if(intCounter >= TX_DUTY_CYCLE) {
			intCounter = 0;
			sendPacketTimer = 1;
		}

		if(sendPacketTimer == 0U) {
			readGPS = 1;
		}

	}
}

void Fatal_Error(void) {
	SdkEvalLedInit(LED1);

	while(1) {
		SdkDelayMs(100);
		SdkEvalLedToggle(LED1);
	}
}

void Appli_Exti_CB(uint16_t GPIO_Pin)
{
	/* In this case the application EXTI event is the button pressed */
	if(GPIO_Pin==GPIO_PIN_13) {
		/* set the button pressed flag */
		but_pressed = 1;
	}
}

/**
 * @brief  Configure all the GPIOs in low power mode.
 * @param  None
 * @retval None
 */
void enterGpioLowPower(void) {
	/* Set all the GPIOs in low power mode (input analog) */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pin = GPIO_PIN_All;

	GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_13); // Button1 Pin
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_All & (~(GPIO_PIN_13 | GPIO_PIN_14)); //SWD Pins
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* de init the SPI */
	SdkEvalSpiDeinit();

	GPS_SPI_DeInit();

	/* keep the EXTI on the button pin */
	SdkEvalPushButtonInit(BUTTON_KEY, BUTTON_MODE_EXTI);

	/* keep the SDN driven */
	SdkEvalM2SGpioInit(M2S_GPIO_SDN, M2S_MODE_GPIO_OUT);

	/* Be sure that it is driving the device to be in shutdown*/
	SdkEvalEnterShutdown();

}

/**
 * @brief  Configure all the GPIOs to be used by the application.
 * @param  None
 * @retval None
 */
void exitGpioLowPower(void) {
	/* Reinit the SDN pin and SPI */
	SdkEvalM2SGpioInit(M2S_GPIO_SDN, M2S_MODE_GPIO_OUT);
	SdkEvalSpiInit();
	EepromCsPinInitialization();

	SdkEvalComInit();

	GPSComInit();
}

/**
 * @brief  This function is used by the S2LPManagementIdentificationRFBoard to set the xtal frequency.
 * @param  xtal_frequency frequency of the oscillator.
 * @retval None
 */
void S2LPRadioSetXtalFrequency(uint32_t xtal_frequency) {
	ST_MANUF_API_set_xtal_freq(xtal_frequency);
}

/**
 * @brief  Blink the application led.
 * @param  times Number of toggles.
 * @retval None
 */
void LedBlink(uint8_t times) {
	SdkEvalLedInit(LED1);
	for(uint8_t i=0;i<times;i++)
	{
		SdkEvalLedToggle(LED1);
		SdkDelayMs(50);
	}
}

/**
 * @brief  System main function.
 * @param  None
 * @retval None
 */
int main(void) {
	/* Instance some variables where the SigFox data of the board are stored */
	uint8_t pac[8];
	uint32_t id;
	uint8_t rcz, retrieverError;

	/* Some variables to store the application data to transmit */
	uint8_t sigfoxPayload[12] = {0};
	uint8_t sigfoxPayloadLen = 0;
	uint8_t downlinkData[8];
	st_sfx_error_t sigfoxError = 0U;

	GPSData_t GPSData = {0};
	struct bme280_data bmeData = {0};

	int16_t altDelta = 0;
	uint8_t speedKmh = 0;

	float curPres = 0.0;
	float startPres = 0.0;
	float tempKelvin = 0.0;

	uint32_t startTime = 0;
	uint32_t ttff = 0;
	bool ttffMeasure = true;

	/* Initialize the hardware */
	HAL_Init();

	ST_LOWLEVEL_UTILS_SetSysClock();

	SdkEvalIdentification();
	SdkEvalM2SGpioInit(M2S_GPIO_SDN, M2S_MODE_GPIO_OUT);
	SdkEvalSpiInit();
	EepromSpiInitialization();

	SdkEvalComInit();

	/* Identify the RF board reading some production data */
	S2LPManagementIdentificationRFBoard();

	/* Put the radio off */
	SdkEvalEnterShutdown();

	/* Give the RF offset to the library */
	ST_MANUF_API_set_freq_offset(S2LPManagementGetOffset());

	/* The low level driver uses the internal RTC as a timer while the STM32 is in low power.
     This function calibrates the RTC using an auxiliary general purpose timer in order to 
     increase its precision. */
	ST_LOWLEVEL_UTILS_TimerCalibration(500);

	/* Initialize the blue push button on the board */
	SdkEvalPushButtonInit(BUTTON_KEY, BUTTON_MODE_EXTI);

	/* Retrieve the SigFox data from the board - ID, PAC and RCZ used */
	retrieverError = enc_utils_retrieve_data(&id, pac, &rcz);

	/* If the retriever returned an error (code different from RETR_OK) the application must not continue */
	if(retrieverError != RETR_OK){
		Fatal_Error();
	}

	/* In case of ETSI we should stuck if RCZ is not 1 */
	if(rcz != 1) {
		Fatal_Error();
	}

	/* RCZ1 - open the SigFox library */
	if(ST_SIGFOX_API_open(ST_RCZ1,(uint8_t*)&id) != 0)
	{
		/* Stuck in case of error */
		Fatal_Error();
	}

	printf("Sampling start pressure...\n");
	startPres = sampleStartPressure(20)/10000.0;
	printf("Sampling done - StartPressure: %f hPa\n", startPres);

	GPSSetup();

	/* Calls IRQ every interval */
	GPSTimerHandle.Instance = TIM7;
	SdkEvalTimersTimConfig_ms(&GPSTimerHandle, GPS_READ_CYCLE * 1000);
	SdkEvalTimersState(&GPSTimerHandle, ENABLE);

	startTime = SdkGetCurrentSysTick();

	/* application main loop */
	while(1) {

		if(readGPS){
			LedBlink(4);
			printf("intCounter = %u/%u \n", intCounter, TX_DUTY_CYCLE);

			switchSPIMode(SPI_MODE_BMx);

			bmeGetData(&bmeData);
			curPres = (float)bmeData.pressure / 10000.0; /* in hPa */
			tempKelvin = ((float)bmeData.temperature / 100.0) + 273.15; /* in °K */

			altDelta = roundf(tempKelvin * preFactor * log(startPres/curPres));
			if(altDelta < 0) altDelta = 0;

			switchSPIMode(SPI_MODE_GPS);

			memset(&GPSData, 0 , sizeof(GPSData));

			while( 1 ) {
				GPSRead(&GPSData);
				if(GPSData.messageType == GPS_IDLE_MESSAGE || GPSData.messageType == GPS_TX_OFF_MESSAGE) {
					break;
				}

				if(hasFix(GPSData)) {

					if(ttffMeasure) {
						ttff = SdkGetCurrentSysTick() - startTime;
						ttffMeasure = false;
					}

					if(GPSData.messageType == GPGGA_MESSAGE) {
						GPSLastFix.time = GPSData.time;
						GPSLastFix.latitude = GPSData.latitude;
						GPSLastFix.longitude = GPSData.longitude;
						GPSLastFix.fixQuality = GPSData.fixQuality;
						GPSLastFix.nSatellites = GPSData.nSatellites;
						GPSLastFix.hdop = GPSData.hdop;
						GPSLastFix.hdopScale = GPSData.hdopScale;
						GPSLastFix.altitude = GPSData.altitude;
						GPSLastFix.altitudeSee = GPSData.altitudeSee;
					}

					if(GPSData.messageType == RMC_MESSAGE) {
						GPSLastFix.time = GPSData.time;
						GPSLastFix.latitude = GPSData.latitude;
						GPSLastFix.longitude = GPSData.longitude;
						GPSLastFix.speed = GPSData.speed;
						GPSLastFix.speedScale = GPSData.speedScale;
					}
				}
			}

			/*
			printf("Latitude: %ld \n", GPSLastFix.latitude);
			printf("Longitude: %ld \n", GPSLastFix.longitude);
			printf("Speed: %f (raw: %d, scale: %d)\n", (float)GPSLastFix.speed/(float)GPSLastFix.speedScale, GPSLastFix.speed, GPSLastFix.speedScale);
			printf("Time: %ld \n", GPSLastFix.time);
			printf("AltitudeDelta: %d \n", altDelta);
			*/

			if(!ttffMeasure) {
				printf("******** TTFF %lu ms \n", ttff);
			}

			readGPS = 0;
		}

		if(but_pressed | sendPacketTimer) {
			LedBlink(8);

			printf("Latitude: %ld \n", GPSLastFix.latitude);
			printf("Longitude: %ld \n", GPSLastFix.longitude);
			printf("Speed: %f (raw: %d, scale: %d)\n", (float)GPSLastFix.speed/(float)GPSLastFix.speedScale, GPSLastFix.speed, GPSLastFix.speedScale);
			printf("Time: %ld \n", GPSLastFix.time);
			printf("AltitudeDelta: %d \n", altDelta);

			speedKmh = roundf( ((float)GPSLastFix.speed/(float)GPSLastFix.speedScale) * 1.852);
			printf("SpeedKmh: %d \n", speedKmh);

			sigfoxPayloadLen = prepareSigfoxPayload(sigfoxPayload, GPSLastFix.latitude, GPSLastFix.longitude, speedKmh, GPSLastFix.time, altDelta);

			if(sigfoxPayloadLen > 12) sigfoxPayloadLen = 12;

			printf("Sigfox Payload: ");
			for(uint8_t i = 0; i < sigfoxPayloadLen; i++) {
				printf("%02X ", sigfoxPayload[i]);
			}
			printf("\n");

			sigfoxError = ST_SIGFOX_API_send_frame(sigfoxPayload, sigfoxPayloadLen, downlinkData, 0, 0);
			if(sigfoxError != 0) {
				/* Stuck in case of error */
				Fatal_Error();
			}

			SdkEvalComInit();

			but_pressed=0;
			sendPacketTimer = 0;

			LedBlink(8);
		}
	}

	return 0;
}

uint32_t sampleStartPressure(uint8_t numSamples) {
	struct bme280_data data = {0};
	uint32_t startPressure = 0;

	BMx_SPI_Init();
	BME_INIT();

	for(uint8_t i = 0; i < numSamples; i++) {
		bmeGetData(&data);
		startPressure += data.pressure;
		SdkDelayMs(250);
	}

	startPressure /= numSamples;

	BMx_SPI_DeInit( );

	return startPressure;
}

void switchSPIMode(uint8_t SPIMode) {

	if(SPIMode == SPI_MODE_GPS) {
		BMx_SPI_DeInit( );
		GPSComInit();
	}else if(SPIMode == SPI_MODE_BMx) {
		GPS_SPI_DeInit();
		BMx_SPI_Init( );
		BME_INIT();
		BMC_INIT();
	}

}

uint8_t prepareSigfoxPayload(uint8_t *dataBuf, uint32_t lat, uint32_t lon, uint8_t speed, uint32_t time, uint16_t altitude) {
	uint8_t byte = 0;

	if(dataBuf == NULL) return 0;

	if(lat <= 0xF ) {
		dataBuf[byte++] = 0x00;
		dataBuf[byte++] = 0x00;
		dataBuf[byte++] = 0x00;

		dataBuf[byte] = (lat << 4) & 0xFF;

	}else if(lat <= 0xFFF) {
		dataBuf[byte++] = 0x00;
		dataBuf[byte++] = 0x00;

		dataBuf[byte++] = (lat >> 4) & 0xFF;
		dataBuf[byte] = (lat << 4) & 0xFF;
	}else if( lat <= 0xFFFFF ) {
		dataBuf[byte++] = 0x00;

		dataBuf[byte++] = (lat >> 12) & 0xFF;
		dataBuf[byte++] = (lat >> 4) & 0xFF;
		dataBuf[byte] = (lat << 4) & 0xFF;

	}else if( lat <= 0x55D4A80 ) { // 0x55D4A80 = Lat 90° 00' 00''
		dataBuf[byte++] = (lat >> 20) & 0xFF;
		dataBuf[byte++] = (lat >> 12) & 0xFF;
		dataBuf[byte++] = (lat >> 4) & 0xFF;
		dataBuf[byte] = (lat << 4) & 0xFF;
	}

	if(lon <= 0xFF) {
		dataBuf[byte++] &= 0xF0;
		dataBuf[byte++] = 0x00;
		dataBuf[byte++] = 0x00;

		dataBuf[byte++] = lon & 0xFF;
	}else if(lon <= 0xFFFF) {
		dataBuf[byte++] &= 0xF0;
		dataBuf[byte++] = 0x00;

		dataBuf[byte++] = (lon >> 8) & 0xFF;
		dataBuf[byte++] = lon & 0xFF;
	}else if(lon <= 0xFFFFFF) {
		dataBuf[byte++] &= 0xF0;

		dataBuf[byte++] = (lon >> 16) & 0xFF;
		dataBuf[byte++] = (lon >> 8) & 0xFF;
		dataBuf[byte++] = lon & 0xFF;

	}else if(lon <= 0xABA9500) { // 0xABA9500 = LON 180° 00' 00''

		dataBuf[byte++] &= (lon >> 24) & 0xFF;
		dataBuf[byte++] = (lon >> 16) & 0xFF;
		dataBuf[byte++] = (lon >> 8) & 0xFF;
		dataBuf[byte++] = lon & 0xFF;
	}

	dataBuf[byte++] = speed & 0xFF;

	dataBuf[byte++] = (time >> 12) & 0xFF;
	dataBuf[byte++] = (time >> 4) & 0xFF;
	dataBuf[byte] = (time << 4) & 0xFF;

	dataBuf[byte++] |= (altitude >> 8) & 0xFF;
	dataBuf[byte++] = altitude & 0xFF;

	return byte;
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval : None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);

	/* Infinite loop */
	while (1)
	{
	}
}
#endif


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
