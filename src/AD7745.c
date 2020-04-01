/**********************************************************************
* @file		AD7745.c
* @brief	Contains a functional hardware-independent driver for the AD7745/6. To use this driver some functions must be implemented before.
*
* @version	2.0
* @date		2. May. 2017
* @author	Leandro Soria
*
**********************************************************************/
#include "AD7745.h"

void AD7745_Reset(void)
{
    uint8_t cmd = AD7745_RESET;
    i2c_write(AD7745_ADDR, (uint8_t*)&cmd, 1);
}

void AD7745_Read(uint8_t subAddr,uint8_t* dataBuffer,uint8_t bytesNumber)
{
    i2c_write(AD7745_ADDR, (uint8_t*)&subAddr, 1);
    i2c_read(AD7745_ADDR, dataBuffer, bytesNumber);
}

void AD7745_Write(uint8_t subAddr, uint8_t* dataBuffer, uint8_t bytesNumber)
{
    uint8_t sendBuffer[10] = {0, };
    uint8_t byte = 0;

    sendBuffer[0] = subAddr;
    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte-1];
    }
    i2c_write(AD7745_ADDR, sendBuffer, bytesNumber+1);
}

uint32_t AD7745_getCapacitance()
{
    uint32_t capacitance = 0;
    uint8_t	 buffer[3] = {0,0,0};

    AD7745_Read(AD7745_ADDR_STATUS, buffer, 1);
// I've decided to ommit this loop, because this wait can make unstable the system
/*
	while((buffer[0] & STATUS_RDYCAP))
    {
    	AD7745_Read(AD7745_ADDR_STATUS, buffer, 1);
    }
*/
    AD7745_Read(AD7745_ADDR_CAP_DATA_H, buffer, 3);
    capacitance = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
    return capacitance;
}

uint32_t AD7745_getTemperature()
{
    uint32_t temperature = 0;
    uint8_t	 buffer[3] = {0, 0, 0};

    AD7745_Read(AD7745_ADDR_STATUS, buffer, 1);
// I've decided to ommit this loop, because this wait can make unstable the system
/*
 	while((buffer[0] & STATUS_RDYVT))
    {
    	AD7745_Read(AD7745_ADDR_STATUS, buffer, 1);
    }
*/
    AD7745_Read(AD7745_ADDR_VT_DATA_H, buffer, 3);
    temperature = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
    return temperature;
}

void AD7745_WriteCapSetupRegister(uint8_t data)
{
	uint8_t write_buffer[2] = {AD7745_ADDR_CAP_SETUP, data};
	i2c_write(AD7745_ADDR, write_buffer, 2);
}

void AD7745_WriteVTSetupRegister(uint8_t data)
{
	uint8_t write_buffer[2] = {AD7745_ADDR_VT_SETUP, data};
	i2c_write(AD7745_ADDR, write_buffer, 2);
}

void AD7745_WriteExcSetupRegister(uint8_t data)
{
	uint8_t write_buffer[2] = {AD7745_ADDR_EXC_SETUP, data};
	i2c_write(AD7745_ADDR, write_buffer, 2);
}

void AD7745_WriteConfigurationRegister(uint8_t data)
{
	uint8_t write_buffer[2] = {AD7745_ADDR_CONFIGURATION, data};
	i2c_write(AD7745_ADDR, write_buffer, 2);
}

void AD7745_WriteCapDacARegister(uint8_t data)
{
	uint8_t write_buffer[2] = {AD7745_ADDR_CAP_DAC_A, data};
	i2c_write(AD7745_ADDR, write_buffer, 2);
}

void AD7745_WriteCapDacBRegister(uint8_t data)
{
	uint8_t write_buffer[2] = {AD7745_ADDR_CAP_DAC_B, data};
	i2c_write(AD7745_ADDR, write_buffer, 2);
}