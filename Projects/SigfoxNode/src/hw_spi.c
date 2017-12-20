/*
 * hw_spi.c
 *
 *  Created on: 12.10.2017
 *      Author: wisbar
 */

#include "hw_spi.h"

void BME_DELAY(uint32_t period);
int8_t BME_SPI_READ(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t BME_SPI_WRITE(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

int8_t BMC_SPI_READ(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
int8_t BMC_SPI_WRITE(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
void BMC_DELAY(u32 period);

void printBmeData(struct bme280_data *comp_data);

#define SPI_READ 0x80
#define SPI_WRITE 0x7F

#define BME280_DATA_INDEX 1
#define BME280_ADDRESS_INDEX 2

#define SPI_BUFFER_LEN 28

static SPI_HandleTypeDef hspi_bmx;

struct bme280_dev bme280;

struct bmm050_t bmm050;

struct bma2x2_t bma2x2;

void BMx_SPI_Init( void ) {

	/* SPI Mode 3: CPOL = 1, CPHA = 1 */
	hspi_bmx.Instance = SPI2;
	hspi_bmx.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi_bmx.Init.Direction = SPI_DIRECTION_2LINES;
	hspi_bmx.Init.Mode = SPI_MODE_MASTER;
	hspi_bmx.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi_bmx.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi_bmx.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi_bmx.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi_bmx.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi_bmx.Init.NSS = SPI_NSS_SOFT;
	hspi_bmx.Init.TIMode = SPI_TIMODE_DISABLE;

	SPI2_CLK_ENABLE();

	if (HAL_SPI_Init(&hspi_bmx) != HAL_OK)
	{
		printf("FATAL ERROR BMC_SPI_Init\n");
		while(1) {}
	}

	/* Initialize SPI2 GPIO Pins */
	GPS_SPI2_IoInit();
}

void BMx_SPI_DeInit( void ) {
	HAL_SPI_DeInit( &hspi_bmx);

    /*##-1- Reset peripherals ####*/
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  /*##-2- Configure the SPI GPIOs */
  BMx_SPI_IoDeInit( );
}

void BMx_SPI_IoDeInit( void ) {
  GPIO_InitTypeDef initStruct={0};

  initStruct.Pin = BMC_NCS_PIN;
  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init ( BMC_NCS_BANK, &initStruct );
  HAL_GPIO_WritePin( BMC_NCS_BANK, BMC_NCS_PIN , GPIO_PIN_SET );

  initStruct.Pin = BME_NCS_PIN;
  HAL_GPIO_WritePin( BME_NCS_BANK, BME_NCS_PIN , GPIO_PIN_SET );

}

int8_t bmcGetAccData(struct bma2x2_accel_data *accData) {
	int8_t rslt = 0;

	rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

	rslt += bma2x2_read_accel_xyz(accData);

	rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);

	return rslt;
}

int8_t bmcGetMagData(struct bmm050_mag_data_s16_t *data) {
	int8_t rslt = 0;

	rslt = bmm050_read_mag_data_XYZ(data);

	return rslt;
}

void bmeGetData(struct bme280_data *data) {

	bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);

	bme280.delay_ms(70);
	bme280_get_sensor_data(BME280_ALL, data, &bme280);

	bme280_set_sensor_mode(BME280_SLEEP_MODE, &bme280);
}

void printBmeData(struct bme280_data *comp_data) {
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

uint32_t BMC_INIT( void ){
	uint32_t rslt = 0;

	bmm050.bus_read = BMC_SPI_READ;
	bmm050.bus_write = BMC_SPI_WRITE;
	bmm050.delay_msec = BMC_DELAY;

	rslt = bmm050_init(&bmm050);

	rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);

	rslt += bmm050_set_data_rate(BMM050_DATA_RATE_30HZ);

	bma2x2.bus_read = BMC_SPI_READ;
	bma2x2.bus_write = BMC_SPI_WRITE;
	bma2x2.delay_msec = BMC_DELAY;

	rslt = bma2x2_init(&bma2x2);

	rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

	rslt += bma2x2_set_bw(0x0E);

	bma2x2_set_range(0x0C);

	rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);

	return rslt;
}

s8 BMC_SPI_READ(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t txBuffer[SPI_BUFFER_LEN];
    uint8_t rxBuffer[SPI_BUFFER_LEN];

    uint8_t i = 0;

    txBuffer[0] = reg_addr | SPI_READ;

    HAL_GPIO_WritePin(BMC_NCS_BANK, BMC_NCS_PIN, GPIO_PIN_RESET);

    status = HAL_SPI_TransmitReceive( &hspi_bmx, txBuffer, rxBuffer, len+1, 100);
    while( hspi_bmx.State == HAL_SPI_STATE_BUSY ) {};
    HAL_GPIO_WritePin(BMC_NCS_BANK, BMC_NCS_PIN, GPIO_PIN_SET);

    SdkDelayMs(5);

    for(i = 0; i < len; i++) {
    	*(reg_data + i) = rxBuffer[i + BME280_DATA_INDEX];
    }

    rslt = (status == HAL_OK ? 0 : 1);

    return rslt;
}

int8_t BMC_SPI_WRITE(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    HAL_StatusTypeDef status = HAL_OK;

    uint8_t buf[SPI_BUFFER_LEN * BME280_ADDRESS_INDEX];
    uint8_t i = 0;
    uint8_t index = 0;

    for(i = 0; i < len; i++) {
    	index = i * BME280_ADDRESS_INDEX;
    	buf[index] = (reg_addr++) & SPI_WRITE;
    	buf[index + BME280_DATA_INDEX] = *(reg_data + i);
    }

    HAL_GPIO_WritePin(BMC_NCS_BANK, BMC_NCS_PIN, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(&hspi_bmx, buf, len*2, 300);
    while( hspi_bmx.State == HAL_SPI_STATE_BUSY ) {};
    HAL_GPIO_WritePin(BMC_NCS_BANK, BMC_NCS_PIN, GPIO_PIN_SET);

    rslt = (status == HAL_OK ? 0 : 1);

    return rslt;
}

void BMC_DELAY(u32 period) {
	SdkDelayMs(period);

	return;
}

int8_t BME_INIT( void ) {
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;

	/* Sensor_0 interface over SPI with native chip select line */
	bme280.dev_id = 0;
	bme280.intf = BME280_SPI_INTF;
	bme280.read = BME_SPI_READ;
	bme280.write = BME_SPI_WRITE;
	bme280.delay_ms = BME_DELAY;

	rslt = bme280_init(&bme280);

	/* Recommended mode of operation: Indoor navigation */
	bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280.settings.filter = BME280_FILTER_COEFF_16;
	bme280.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &bme280);

	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);
	//rslt = bme280_set_sensor_mode(BME280_SLEEP_MODE, &bme280);

	return rslt;
}

int8_t BME_SPI_READ(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t txBuffer[SPI_BUFFER_LEN];
    uint8_t rxBuffer[SPI_BUFFER_LEN];

    uint8_t i = 0;

    txBuffer[0] = reg_addr | SPI_READ;

    HAL_GPIO_WritePin(BME_NCS_BANK, BME_NCS_PIN, GPIO_PIN_RESET);

    status = HAL_SPI_TransmitReceive( &hspi_bmx, txBuffer, rxBuffer, len+1, 100);
    while( hspi_bmx.State == HAL_SPI_STATE_BUSY ) {};
    HAL_GPIO_WritePin(BME_NCS_BANK, BME_NCS_PIN, GPIO_PIN_SET);

    SdkDelayMs(5);

    for(i = 0; i < len; i++) {
    	*(reg_data + i) = rxBuffer[i + BME280_DATA_INDEX];
    }

    rslt = (status == HAL_OK ? 0 : 1);

    return rslt;
}

int8_t BME_SPI_WRITE(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    HAL_StatusTypeDef status = HAL_OK;

    uint8_t buf[SPI_BUFFER_LEN * BME280_ADDRESS_INDEX];
    uint8_t i = 0;
    uint8_t index = 0;

    for(i = 0; i < len; i++) {
    	index = i * BME280_ADDRESS_INDEX;
    	buf[index] = (reg_addr++) & SPI_WRITE;
    	buf[index + BME280_DATA_INDEX] = *(reg_data + i);
    }

    HAL_GPIO_WritePin(BME_NCS_BANK, BME_NCS_PIN, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(&hspi_bmx, buf, len*2, 300);
    while( hspi_bmx.State == HAL_SPI_STATE_BUSY ) {};
    HAL_GPIO_WritePin(BME_NCS_BANK, BME_NCS_PIN, GPIO_PIN_SET);

    rslt = (status == HAL_OK ? 0 : 1);

    return rslt;
}

void BME_DELAY(uint32_t period) {

	SdkDelayMs(period);

	return;
}

int8_t streamBmeDataNormalMode( void ) {
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280.settings.filter = BME280_FILTER_COEFF_16;
	bme280.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &bme280);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);

	printf("Temperature, Pressure, Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		bme280.delay_ms(70);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280);
		printBmeData(&comp_data);
	}

	return rslt;
}
