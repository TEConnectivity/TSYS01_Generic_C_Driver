/**
 * \file tsys01.h
 *
 * \brief TSYS01 Temperature sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 *
 * \asf_license_stop
 *
 */

#ifndef TSYS01_H_INCLUDED
#define TSYS01_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

enum tsys01_address {
	tsys01_i2c_address_csb_1,
	tsys01_i2c_address_csb_0
};

enum tsys01_status {
	tsys01_status_ok,
	tsys01_status_no_i2c_acknowledge,
	tsys01_status_i2c_transfer_error,
	tsys01_status_crc_error
};
	
// Functions

/**
 * \brief Configures the SERCOM I2C master to be used with the tsys01 device.
 */
void tsys01_init(void);

/**
 * \brief Configures TSYS01 I2C address to be used depending on HW configuration
 *
 * \param[in] tsys01_address : TSYS01 I2C address
 *
 */
void tsys01_set_address( enum tsys01_address address);

/**
 * \brief Check whether TSYS01 device is connected
 *
 * \return bool : status of TSYS01
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool tsys01_is_connected(void);

/**
 * \brief Reset the TSYS01 device
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status tsys01_reset(void);

/**
 * \brief Reads the temperature ADC value and compute the degree Celsius one.
 *
 * \param[out] float* : Celsius Degree temperature value
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
 */
enum tsys01_status tsys01_read_temperature(float *);

#endif /* TSYS01_H_INCLUDED */