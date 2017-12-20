/*
 * hw_spi.h
 *
 *  Created on: 12.10.2017
 *      Author: wisbar
 */

#ifndef SIGFOXNODE_INC_HW_SPI_H_
#define SIGFOXNODE_INC_HW_SPI_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "SDK_EVAL_Config.h"
#include "GPS.h"
#include "bme280.h"
#include "bmm050.h"
#include "bma2x2.h"

#define BMC_NCS_PIN GPIO_PIN_9
#define BMC_NCS_BANK GPIOC

#define BME_NCS_PIN GPIO_PIN_8
#define BME_NCS_BANK GPIOC

void BMx_SPI_Init( void );

void BMx_SPI_DeInit( void );
void BMx_SPI_IoDeInit( void );

int8_t BME_INIT( void );
uint32_t BMC_INIT( void );

void bmeGetData(struct bme280_data *data);

int8_t bmcGetMagData(struct bmm050_mag_data_s16_t *data);
int8_t bmcGetAccData(struct bma2x2_accel_data *accData);

int8_t streamBmeDataNormalMode( void );

#endif /* SIGFOXNODE_INC_HW_SPI_H_ */
