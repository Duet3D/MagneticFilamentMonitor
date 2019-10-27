/*
 * AS5601.cpp
 *
 * Created: 06/05/2017 13:47:44
 *  Author: tony@think3dprint3d.com
 
 Based on the AS5601 data sheet
 */

#include "ecv.h"

#ifdef __ECV__
#define __attribute__(_x)
#define __volatile__
#pragma ECV noverifyincludefiles
#endif

#include <avr/interrupt.h>
#include <avr/io.h>

#ifdef __ECV__
#pragma ECV verifyincludefiles
#undef cli
#undef sei
extern void cli();
extern void sei();
#endif

#include "USI_TWI_Master.h"
#include "AS5601.h"

#define MESSAGEBUF_SIZE       4 //Address + Register + 2 byte is the largest read/write in 1 operation
uint8_t messageBuf[MESSAGEBUF_SIZE];

//I2C communications

/*
Used to set the register on the AS5601 for an upcoming read.
*/
bool AS5601_SetRegister(uint8_t registerAddress)
{
	//construct the message
	messageBuf[0] = (AS5601_ADDRESS << TWI_ADR_BITS) | (0u << TWI_READ_BIT);
	messageBuf[1] = registerAddress;
	//write operation
	return USI_TWI_Start_Read_Write(messageBuf, 2);
}

uint8_t AS5601_Read8(uint8_t registerAddress)
{
	if (!AS5601_SetRegister(registerAddress))
	{
		return 0;																	 // error
	}
	
	messageBuf[0] = (AS5601_ADDRESS << TWI_ADR_BITS) | (1u << TWI_READ_BIT);		// construct the message
	const bool ret = USI_TWI_Start_Read_Write(messageBuf, 2);						// read operation
	USI_TWI_Master_Stop();
	return (ret) ? messageBuf[1] : 0;
}

uint16_t AS5601_Read16(uint8_t registerAddress)
{
	if (!AS5601_SetRegister(registerAddress))
	{
		return 0;																	// error
	}

	messageBuf[0] = (AS5601_ADDRESS << TWI_ADR_BITS) | (1u << TWI_READ_BIT);		// construct the message
	const bool ret = USI_TWI_Start_Read_Write(messageBuf, 3);						// read operation
	USI_TWI_Master_Stop();
	return (ret) ? (uint16_t)(((uint16_t)messageBuf[1] << 8u) | messageBuf[2]) : 0;
}


// As per read16, just do not set the target register (needs to be set with Set Register separately)
uint16_t AS5601_ReadFast16(uint8_t registerAddress)
{
	messageBuf[0] = (AS5601_ADDRESS << TWI_ADR_BITS) | (1u << TWI_READ_BIT);		// construct the message

	const bool ret = USI_TWI_Start_Read_Write(messageBuf, 3);						// read operation
	return (ret) ? (uint16_t)(((uint16_t)messageBuf[1] << 8) | messageBuf[2]) : 0;
}

bool AS5601_Write8(uint8_t registerAddress, uint8_t value)
{
	// Construct the message
	messageBuf[0] = (AS5601_ADDRESS << TWI_ADR_BITS) | (0u << TWI_READ_BIT);
	messageBuf[1] = registerAddress;
	messageBuf[2] = value;
	
	const bool b = USI_TWI_Start_Read_Write(messageBuf, 3);
	USI_TWI_Master_Stop();
	return b;
}

bool AS5601_Write16(uint8_t registerAddress, uint16_t value )
{
	// Construct the message
	messageBuf[0] = (AS5601_ADDRESS << TWI_ADR_BITS) | (0u << TWI_READ_BIT);
	messageBuf[1] = registerAddress;
	messageBuf[2] = (uint8_t)(value >> 8);
	messageBuf[3] = (uint8_t)value;

	const bool b = USI_TWI_Start_Read_Write(messageBuf, 4);
	USI_TWI_Master_Stop();
	return b;
}

/*
Run this first
Connect over I2C and configure the AS5601,sets the config values to the defaults.
*/

bool AS5601_Initialise(uint16_t config)
{
	USI_TWI_Master_Initialise();
	return AS5601_Write16(RegCONFA, config);
}

/*
The CONF register supports customizing the AS5601 the following information is returned:
Bit position	Description
	1:0			Power Mode	00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
	3:2			Hysteresis	00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
	9:8			Slow Filter	00 = 16x (1); 01 = 8x; 10 = 4x; 11 = 2x
	12:10		Fast Filter Threshold 000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101 = 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
	13			Watchdog Timer 0 = OFF, 1 = ON (automatic entry into LPM3 low-power mode enabled)
*/
uint16_t AS5601_GetConfig(void)
{
	return AS5601_Read16(RegCONFA);
}

/*
bool AS5601_SetConfig(uint16_t config)
{
	return AS5601_Write16(CONFA,config);
}
*/

/*
POWER MODE
Three low-power modes are available. In a low-power mode, the fast filter is automatically disabled,
because there is no need for a fast settling time if the output refresh is as fast as the polling cycles.

Default 00 = NOM
*/
bool AS5601_SetPowerMode(uint8_t pm)
{
	const uint8_t confb_current = (AS5601_Read8(RegCONFB) & 0x0Fu);
	return AS5601_Write8(RegCONFB,((confb_current & 0x0Cu) | pm));
}

/*
HYSTERESIS
To suppress spurious toggling of the output when the magnet is not moving, a 1 to 3 LSB hysteresis of the 12-bit resolution
can be enabled

Default 00 = OFF
*/
bool AS5601_SetHysteresis(uint8_t hyst)
{
	const uint8_t confb_current = (AS5601_Read8(RegCONFB) & 0x0Fu);
	return AS5601_Write8(RegCONFB,((confb_current & 0x03u) | hyst << 2u));
}

/*
SLOW FILTER
If the fast filter is OFF, the step output response is controlled by
the slow linear filter.
	SF			Step Response		Max. RMS Output Noise
				Delay (ms)			(1 Sigma) (Degree)
				
	16X, 00			2.2					0.015
	 8X, 01			1.1					0.021
	 4X, 10			0.55				0.030
	 2X, 11			0.286				0.043

Default 2X,11
*/
bool AS5601_SetSlowFilter(uint8_t sf)
{
	const uint8_t confa_current = (AS5601_Read8(RegCONFA) & 0x3Fu);
	return AS5601_Write8(RegCONFA, ((confa_current & 0x3Cu) | sf));
}

/*
FAST FILTER
For a fast step response and low noise after settling, the fast filter can be enabled.
The fast filter works only if the input variation is greater than the fast filter
threshold, otherwise the output response is determined only by the slow filter.

						Fast Filter Threshold
	FTH			Slow-to-Fast Filter		Fast-to-Slow Filter
	
	FTH_SFO			/							/
	FTH_1			6							1
	FTH_2			7							1
	FTH_3			9							1
	FTH_4			18							2
	FTH_5			21							2
	FTH_6			24							2
	FTH_7			10							4


Default FTH_4
*/
bool AS5601_SetFastFilter(uint8_t fth)
{
	const uint8_t confa_current = (AS5601_Read8(RegCONFA) & 0x3Fu);
	return AS5601_Write8(RegCONFA,((confa_current & 0x23u) | fth << 2u));
}

/*
WATCHDOG TIMER
The watchdog timer allows saving power by switching into LMP3 if the angle stays within
the watchdog threshold of 4 LSB for at least one minute,

Default OFF
*/
bool AS5601_SetWatchdog(uint8_t wd)
{
	const uint8_t confa_current = (AS5601_Read8(RegCONFA) & 0x3Fu);
	return AS5601_Write8(RegCONFA,((confa_current & 0x1Fu) | wd << 5u));
}

/*
MH AGC minimum gain overflow, magnet too strong
ML AGC maximum gain overflow, magnet too weak
MD Magnet was detected
*/
uint8_t AS5601_GetStatus(void)
{
	return AS5601_Read8(RegSTATUS) & (StatusMD | StatusMH | StatusML);
}

// The MAGNITUDE register indicates the magnitude value of the internal CORDIC output.
uint16_t AS5601_GetMagnitude(void)
{
	return AS5601_Read16(RegMAGNITUDEA) & 0x0FFFu;
}	

/*
For the most robust performance, the gain value should be in the center of its range.
In 5V operation, the AGC range is 0-255 counts. The AGC range is reduced to 0-128 counts in 3.3V mode.
*/
uint8_t AS5601_GetAGC(void)
{
	return AS5601_Read8(RegAGC);
}			

/*
The RAW ANGLE register contains the unmodified angle.
The zero adjusted and filtered output value is available in the ANGLE register.
Note(s): The ANGLE register has a 10-LSB hysteresis at the limit of the 360 degree range to avoid discontinuity points or toggling of the output within one rotation.
*/
uint16_t AS5601_GetRawAngle(void)
{
	return AS5601_ReadFast16(RegRAWANGLEA) & 0x0FFFu;
}

/*
The zero adjusted and filtered output value is available in the ANGLE register.
*/
uint16_t AS5601_GetAngle(void)
{
	return AS5601_Read16(RegANGLEA) & 0x0FFFu;
}

/*
It sets the current ANGLE to be "0"
*/
void AS5601_SetCurrentZeroPosition()
{
	AS5601_SetRegister(RegRAWANGLEA);
	uint16_t rawAngle = AS5601_GetRawAngle();
	AS5601_SetRegister(RegZPOSA);
	AS5601_Write16(RegZPOSA, rawAngle);
}

/*
With the setting ABN(3:0) it is possible to configure the number of angle positions and
the update frequency:
Output Positions and Update Rate
0000 : 8 (61 Hz)
0001 : 16 (122 Hz)
0010 : 32 (244 Hz)
0011 : 64 (488 Hz)
0100 : 128 (976 Hz)
0101 : 256 (1.9 kHz)
0110 : 512 (3.9 kHz)
0111 : 1024 (7.8 kHz)
others : 2048 (15.6 kHz))
*/
void AS5601_SetABN(uint8_t abn_value)
{
	AS5601_Write8(RegABN,abn_value);
}

/*
Pass through the I2C connection state
*/
uint8_t AS5601_GetTWIStateInfo()
{
	return USI_TWI_Get_State_Info();
}

// End
