/**********************************************************************
* @file		i2c.h
* @brief	API for the I2C driver needed to communicate with the AD7745/6. 
*           It's user responsability to implement the following functions.
*
* @version	2.0
* @date		2. May. 2017
* @author	Leandro Soria
*
**********************************************************************/

#include <stdint.h>

/**
 * @brief Writes data to a slave device.
 *
 * @param slave_address - Address of the slave device.
 * @param data_buffer - Pointer to a buffer storing the transmission data.
 * @param data_size - Number of bytes to write.
 *
 * @return None.
*/
void i2c_write(uint8_t slave_address, uint8_t* data_buffer, uint8_t data_size);

/**
 * @brief Reads data from a slave device.
 *
 * @param slave_address - Address of the slave device.
 * @param data_buffer - Pointer to a buffer that will store the received data.
 * @param data_size - Number of bytes to read.
 *
 * @return None.
*/
void i2c_read(uint8_t slave_address, uint8_t* data_buffer, uint8_t data_size);