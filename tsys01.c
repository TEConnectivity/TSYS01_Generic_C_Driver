/**
 * \file tsys01.c
 *
 * \brief TSYS01 Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to tsys01 datasheet :
 * http://www.meas-spec.com/downloads/TSYS01_Digital_Temperature_Sensor.pdf
 *
 */

#include "tsys01.h"

 /**
  * The header "i2c.h" has to be implemented for your own platform to 
  * conform the following protocol :
  *
  * enum i2c_transfer_direction {
  * 	I2C_TRANSFER_WRITE = 0,
  * 	I2C_TRANSFER_READ  = 1,
  * };
  * 
  * enum status_code {
  * 	STATUS_OK           = 0x00,
  * 	STATUS_ERR_OVERFLOW	= 0x01,
  *		STATUS_ERR_TIMEOUT  = 0x02,
  * };
  * 
  * struct i2c_master_packet {
  * 	// Address to slave device
  * 	uint16_t address;
  * 	// Length of data array
  * 	uint16_t data_length;
  * 	// Data array containing all data to be transferred
  * 	uint8_t *data;
  * };
  * 
  * void i2c_master_init(void);
  * enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet);
  * enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet);
  * enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);
  */
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constants

// TSYS01 device address
#define TSYS01_ADDR_CSB_1									0x76 //0b1110110
#define TSYS01_ADDR_CSB_0									0x77 //0b1110111

// TSYS01 device commands
#define TSYS01_RESET_COMMAND								0x1E
#define TSYS01_START_ADC_CONVERSION							0x48
#define TSYS01_READ_ADC_TEMPERATURE							0x00
#define PROM_ADDRESS_READ_ADDRESS_0							0xA0
#define PROM_ADDRESS_READ_ADDRESS_1							0xA2
#define PROM_ADDRESS_READ_ADDRESS_2							0xA4
#define PROM_ADDRESS_READ_ADDRESS_3							0xA6
#define PROM_ADDRESS_READ_ADDRESS_4							0xA8
#define PROM_ADDRESS_READ_ADDRESS_5							0xAA
#define PROM_ADDRESS_READ_ADDRESS_6							0xAC
#define PROM_ADDRESS_READ_ADDRESS_7							0xAE
#define PROM_ELEMENTS_NUMBER								8

// Coefficients for temperature computation
#define COEFF_MUL_0											(float)(-1.5)
#define COEFF_MUL_1											(float)(1)
#define COEFF_MUL_2											(float)(-2)
#define COEFF_MUL_3											(float)(4)
#define COEFF_MUL_4											(float)(-2)

#define TSYS01_CONVERSION_TIME								10000

// Static functions
static enum tsys01_status tsys01_write_command(uint8_t);
static enum tsys01_status tsys01_read_eeprom_coeff(uint8_t, uint16_t*);
static enum tsys01_status tsys01_read_eeprom(void);
static enum tsys01_status tsys01_conversion_and_read_adc(uint32_t *);
static bool tsys01_crc_check (uint16_t *n_prom);

static const float coeff_mul[5] = {COEFF_MUL_0, COEFF_MUL_1, COEFF_MUL_2, COEFF_MUL_3, COEFF_MUL_4 };
static uint16_t eeprom_coeff[PROM_ELEMENTS_NUMBER];

uint8_t tsys01_i2c_address = TSYS01_ADDR_CSB_0;

// Default value to ensure coefficients are read before converting temperature
bool tsys01_coeff_read = false;

/**
 * \brief Configures the SERCOM I2C master to be used with the TSYS01 device.
 */
void tsys01_init(void)
{
	i2c_master_init();
}

/**
 * \brief Configures TSYS01 I2C address to be used depending on HW configuration
 *
 * \param[in] address : TSYS01 I2C address
 *
 */
void tsys01_set_address( enum tsys01_address address)
{
	if(address == tsys01_i2c_address_csb_1)
		tsys01_i2c_address = TSYS01_ADDR_CSB_1;
	else
		tsys01_i2c_address = TSYS01_ADDR_CSB_0;
}

/**
 * \brief Check whether TSYS01 device is connected
 *
 * \return bool : status of TSYS01
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool tsys01_is_connected(void)
{
	enum status_code i2c_status;
	
	struct i2c_master_packet transfer = {
		.address     = tsys01_i2c_address,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status != STATUS_OK)
		return false;
	
	return true;
}
	
/**
 * \brief Reset the TSYS01 device
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status  tsys01_reset(void)
{
	return tsys01_write_command(TSYS01_RESET_COMMAND);
}

/**
 * \brief Writes the TSYS01 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status tsys01_write_command( uint8_t cmd)
{
	enum status_code i2c_status;
	uint8_t data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = tsys01_i2c_address,
		.data_length = 1,
		.data        = data,
	};
	
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys01_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys01_status_i2c_transfer_error;
	
	return tsys01_status_ok;
}

/**
 * \brief Reads the tsys01 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status tsys01_read_eeprom_coeff(uint8_t command, uint16_t *coeff)
{
	enum tsys01_status status;
	enum status_code i2c_status;
	uint8_t buffer[2];
	
	buffer[0] = 0;
	buffer[1] = 0;

	struct i2c_master_packet read_transfer = {
		.address     = tsys01_i2c_address,
		.data_length = 2,
		.data        = buffer,
	};
	
	// Send the conversion command
	status = tsys01_write_command(command);
	if(status != tsys01_status_ok)
		return status;
	
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys01_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys01_status_i2c_transfer_error;
		
	*coeff = (buffer[0] << 8) | buffer[1];
	
	return tsys01_status_ok;	
}

/**
 * \brief Reads the tsys01 EEPROM coefficients to store them for computation.
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
 */
enum tsys01_status tsys01_read_eeprom(void)
{
	enum tsys01_status status;
	uint8_t i;

	// Read all coefficients from EEPROM	
	for( i=0 ; i<PROM_ELEMENTS_NUMBER ; i++)
	{
		status = tsys01_read_eeprom_coeff( PROM_ADDRESS_READ_ADDRESS_0 + i*2, eeprom_coeff+i);
		if(status != tsys01_status_ok)
			return status;
	}
	
	// CRC check
	if( !tsys01_crc_check( eeprom_coeff ) )
		return tsys01_status_crc_error;
	
	tsys01_coeff_read = true;
	
	return tsys01_status_ok;
}

/**
 * \brief Reads the temperature ADC value
 *
 * \param[out] uint32_t* : Temperature ADC value.
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
 */
static enum tsys01_status tsys01_conversion_and_read_adc(uint32_t *adc)
{
	enum tsys01_status status;
	enum status_code i2c_status;
	uint8_t buffer[3];
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    struct i2c_master_packet read_transfer = {
		.address     = tsys01_i2c_address,
		.data_length = 3,
		.data        = buffer,
	};

	status = tsys01_write_command(TSYS01_START_ADC_CONVERSION);
	// Wait for conversion
	delay_ms(TSYS01_CONVERSION_TIME/1000);
	if( status != tsys01_status_ok)
		return status;

	// Send the read command
	status = tsys01_write_command(TSYS01_READ_ADC_TEMPERATURE);
	if( status != tsys01_status_ok)
		return status;
	
    i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys01_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys01_status_i2c_transfer_error;

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
	
	return status;
}

/**
 * \brief Reads the temperature ADC value and compute the degree Celsius one.
 *
 * Perform algorithm computation based on adc 16 bits ( 24-bits value is divided by 256)<BR>
 * \verbatim
 *  T (degC) =     (-2) * k4 * 10e-21 * adc^4 +
 *                    4 * k3 * 10e-16 * adc^3 +
 *                 (-2) * k2 * 10e-11 * adc^2 +
 *                    1 * k1 * 10e-6  * adc +
 *               (-1.5) * k0 * 10e-2
 * \endverbatim
 * 
 * Factored into
 * \verbatim
 *  T (degC) = 10e-2.( a.k0 + 10e1 * 10e-5.adc.( b.k1 + 10e-5.adc ( c.k2 + 10e-5.adc.( d.k3 + 10e-5.adc.e.k4 ) ) ) )
 * \endverbatim
 *
 * \param[out] float* : Celsius Degree temperature value
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
  */

enum tsys01_status tsys01_read_temperature(float *temperature)
{
	enum tsys01_status status = tsys01_status_ok;
	uint32_t adc;
	uint8_t i;
	float temp = 0;
	
	// If first time temperature is requested, get EEPROM coefficients
	if( tsys01_coeff_read == false )
		status = tsys01_read_eeprom();
	if( status != tsys01_status_ok)
		return status;
		
	status = tsys01_conversion_and_read_adc(&adc);
	if( status != tsys01_status_ok)
		return status;

	adc /= 256;

	for( i=4 ; i>0 ; i--)
	{
		temp += coeff_mul[i] * eeprom_coeff[1+(4-i)]; // eeprom_coeff[1+(4-i)] equiv. ki
		temp *= (float)adc/100000;
	}
	temp *= 10;
	temp += coeff_mul[0] * eeprom_coeff[5];
	temp /= 100;
	
	*temperature = temp;
	
	return status;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool tsys01_crc_check (uint16_t *n_prom)
{
	uint8_t cnt;
	uint16_t sum = 0 ;

	for( cnt = 0 ; cnt < PROM_ELEMENTS_NUMBER ; cnt++ )
		// Sum each byte of the coefficients
		sum += ( ( n_prom[cnt]>>8 ) + ( n_prom[cnt] & 0xFF ) ) ;
	
	return  ( sum & 0xFF == 0 );
}

#ifdef __cplusplus
}
#endif