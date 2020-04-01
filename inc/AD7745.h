/**********************************************************************
* @file		AD7745.h
* @brief	Contains a functional hardware-independent driver for the AD7745/6. To use this driver some functions must be implemented before.
*
* @version	2.0
* @date		2. May. 2017
* @author	Leandro Soria
*
**********************************************************************/
#ifndef __AD7745_DRIVER_H__
#define	__AD7745_DRIVER_H__

#include <stdint.h>

//********************************************************************************//
// AD7745 main adresses (You must write these ones on the I2C bus first if you want to perform an read or write operation)
#define AD7745_ADDR							0x48	// 0b|1001000|	7-bit address
#define AD7745_ADDR_WRITE   				0x90	// 0b|1001000|0	7-bit address + Write Bit
#define AD7745_ADDR_READ   					0x91	// 0b|1001000|1	7-bit address + Read Bit
#define AD7745_CHIP_ID    					0x00
#define AD7745_RESET                    	0xBF
//********************************************************************************//

//********************************************************************************//
// AD7745 registers sub-adresses
#define AD7745_ADDR_STATUS   				0x00
#define AD7745_ADDR_CAP_DATA_H   			0x01
#define AD7745_ADDR_CAP_DATA_M    			0x02
#define AD7745_ADDR_CAP_DATA_L    			0x03
#define AD7745_ADDR_VT_DATA_H   			0x04
#define AD7745_ADDR_VT_DATA_M   			0x05
#define AD7745_ADDR_VT_DATA_L   			0x06
#define AD7745_ADDR_CAP_SETUP  				0x07
#define AD7745_ADDR_VT_SETUP   				0x08
#define AD7745_ADDR_EXC_SETUP   			0x09
#define AD7745_ADDR_CONFIGURATION   		0x0A
#define AD7745_ADDR_CAP_DAC_A   			0x0B
#define AD7745_ADDR_CAP_DAC_B   			0x0C
#define AD7745_ADDR_CAP_OFFSET_H   			0x0D
#define AD7745_ADDR_CAP_OFFSET_L   			0x0E
#define AD7745_ADDR_CAP_GAIN_H   			0x0F
#define AD7745_ADDR_CAP_GAIN_L   			0x10
#define AD7745_ADDR_VOLTAGE_GAIN_H   		0x11
#define AD7745_ADDR_VOLTAGE_GAIN_L			0x12
//********************************************************************************//

//********************************************************************************//
// AD7745 status register bit masks
#define STATUS_EXCERR						0b00001000
#define STATUS_RDY							0b00000100
#define STATUS_RDYVT						0b00000010
#define STATUS_RDYCAP						0b00000001
//********************************************************************************//

//********************************************************************************//
// AD7745 capacitive channel setup bit masks 
// CAPEN = 1 enables capacitive channel for single conversion, continuous conversion, or calibration.
#define CAPSETUP_CAPEN_OFF 					0b00000000
#define CAPSETUP_CAPEN_ON 					0b10000000
// CIN2 = 1 switches the internal multiplexer to the second capacitive input on the AD7746.
#define CAPSETUP_CIN2_OFF					0b00000000
#define CAPSETUP_CIN2_ON					0b01000000
// DIFF = 1 sets differential mode on the selected capacitive input.
#define CAPSETUP_CAPDIFF_OFF 				0b00000000
#define CAPSETUP_CAPDIFF_ON 				0b00100000
// CAPCHOP = 1 approximately doubles the capacitive channel conversion times and slightly
// improves the capacitive channel noise performance for the longest conversion times.
#define CAPSETUP_CAPCHOPP_OFF 				0b00000000
#define CAPSETUP_CAPCHOPP_ON 				0b00000001
//********************************************************************************//

//********************************************************************************//
// AD7745 Voltage/Temperature channel setup bit masks
// VTEN = 1 enables voltage/temperature channel for single conversion, continuous conversion, or calibration.
#define VTSETUP_VTEN_OFF					0b00000000
#define VTSETUP_VTEN_ON						0b10000000
// Voltage/Temperature channel input  configuration
// Internal temperature sensor
#define VTSETUP_VTMD_INTERNAL_TEMP			0b00000000
// External temperature sensor diode
#define VTSETUP_VTMD_EXTERNAL_TEMP			0b00100000
// VDD monitor
#define VTMD_VDD_MONITOR					0b01000000
// External voltage input (VIN)
#define VTSETUP_VTMD_EXTERNAL_VOLTAGE 		0b01100000
// EXTREF = 0 selects the on-chip internal reference. The internal reference must be used with the internal temperature sensor for proper operation.
#define VTSETUP_EXTREF_OFF					0b00000000
// EXTREF = 1 selects an external reference voltage connected to REFIN(+), REFIN(–) for the voltage input or the VDD monitor.
#define VTSETUP_EXTREF_ON					0b00010000
// VTSHORT = 1 internally shorts the voltage/temperature channel input for test purposes.
#define VTSETUP_VTSHORT_OFF					0b00000000
#define VTSETUP_VTSHORT_ON					0b00000010
// VTCHOP = 1 sets internal chopping on the voltage/temperature channel. 
// The VTCHOP bit must be set to 1 for the specified voltage/temperature channel performance.
#define VTSETUP_VTCHOP_OFF					0b00000000
#define VTSETUP_VTCHOP_ON					0b00000001
//********************************************************************************//

//********************************************************************************//
// AD7745 Capacitive channel excitation setup bit masks
// The CLKCTRL bit should be set to 0 for the specified AD7745/AD7746 performance.
// CLKCTRL = 1 decreases the excitation signal frequency and the modulator clock frequency by factor of 2.
// This also increases the conversion time on all channels (capacitive, voltage, and temperature) by a factor of 2.
#define EXCSETUP_CLKCTRL_OFF				0b00000000
#define EXCSETUP_CLKCTRL_ON					0b10000000
// When EXCON = 0, the excitation signal is present on the output only during capacitance channel conversion. 
// When EXCON = 1, the excitation signal is present on the output during both capacitance and voltage/temperature conversion.
#define EXCSETUP_EXCON_OFF					0b00000000
#define EXCSETUP_EXCON_ON					0b01000000
// EXCB = 1 enables EXCB pin as the excitation output.
#define EXCSETUP_EXCB_OFF					0b00000000
#define EXCSETUP_EXCB_ON					0b00100000
//nEXCB = 1 enables EXCB pin as the inverted excitation output. 
// Only one of the EXCB or the EXCB bits should be set for proper operation.
#define EXCSETUP_nEXCB_OFF					0b00000000
#define EXCSETUP_nEXCB_ON					0b00010000
//EXCA = 1 enables EXCA pin as the excitation output.
#define EXCSETUP_EXCA_OFF					0b00000000
#define EXCSETUP_EXCA_ON					0b00001000
// EXCA = 1 enables EXCA pin as the inverted excitation output.
// Only one of the EXCA or the EXCA bits should be set for proper operation.
#define EXCSETUP_nEXCA_OFF					0b00000000
#define EXCSETUP_nEXCA_ON					0b00000100
// Excitation Voltage Level
#define EXCSETUP_EXCLVL_VDD_8   			0b00000000
#define EXCSETUP_EXCLVL_VDD_4    			0b00000001
#define EXCSETUP_EXCLVL_3VDD_8				0b00000010
#define EXCSETUP_EXCLVL_VDD_2				0b00000011
//********************************************************************************//

//********************************************************************************//
// AD7745 Converter update rate and mode of operation setup
// Voltage/temperature channel digital filter setup—conversion time/update rate setup. 
// The conversion times in this table are valid for the CLKCTRL = 0 in the EXC SETUP register. 
// The conversion times are longer by a factor of two for the CLKCTRL = 1.
#define CONF_VTF_20_MS						0b00000000
#define CONF_VTF_32_MS						0b01000000
#define CONF_VTF_62_MS						0b10000000
#define CONF_VTF_122_MS						0b11000000
// Capacitive channel digital filter setup—conversion time/update rate setup. 
// The conversion times in this table are valid for the CLKCTRL = 0 in the EXC SETUP register. 
// The conversion times are longer by factor of two for the CLKCTRL = 1.
#define CONF_CAPF_11_MS						0b00000000
#define CONF_CAPF_12_MS						0b00001000
#define CONF_CAPF_20_MS						0b00010000
#define CONF_CAPF_38_MS						0b00011000
#define CONF_CAPF_62_MS						0b00100000
#define CONF_CAPF_77_MS						0b00101000
#define CONF_CAPF_92_MS						0b00110000
#define CONF_CAPF_109_MS					0b00111000
// Converter mode of operation setup.
#define CONF_MD_IDLE						0b00000000
#define CONF_MD_CONTINUOUS_CONVERSION		0b00000001
#define CONF_MD_SINGLE_CONVERSION			0b00000010
#define CONF_MD_POWER_DOWN					0b00000011
#define CONF_MD_CAP_OFFSET_CALIBRATION 		0b00000101
#define	CONF_MD_CAP_GAIN_CALIBRATION		0b00000110
//********************************************************************************//

//********************************************************************************//
// AD7745 Capacitive DAC setup
// DACAENA = 1 connects capacitive DACA to the positive capacitance input.
#define CAPDACA_DACAENA_OFF					0b00000000
#define CAPDACA_DACAENA_ON					0b10000000
//********************************************************************************//

//********************************************************************************//
// AD7745 Capacitive DAC setup
// DACBENB = 1 connects capacitive DACB to the negative capacitance input.
#define CAPDACB_DACBENB_OFF 				0b00000000
#define CAPDACB_DACBENB_ON					0b10000000
//********************************************************************************//

void AD7745_Init();
void AD7745_Reset();

void AD7745_Read(uint8_t subAddr,uint8_t* dataBuffer,uint8_t bytesNumber);
void AD7745_Write(uint8_t subAddr, uint8_t* dataBuffer, uint8_t bytesNumber);

uint32_t AD7745_getCapacitance();
uint32_t AD7745_getTemperature();
    
void AD7745_WriteCapSetupRegister(uint8_t data);
void AD7745_WriteVTSetupRegister(uint8_t data);
void AD7745_WriteExcSetupRegister(uint8_t data);
void AD7745_WriteConfigurationRegister(uint8_t data);
void AD7745_WriteCapDacARegister(uint8_t data);
void AD7745_WriteCapDacBRegister(uint8_t data);

#endif  // __AD7745_DRIVER_H__