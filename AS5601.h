/*
 * AS5601.h
 *
 * Created: 03/05/2017 00:20:16
 *  Author: tony@think3dprint3d.com
 
 Based on the AS5601 data sheet
 */ 

#ifndef AS5601
#define AS5601

//address
#define		AS5601_ADDRESS	0x36u

//Configuration Registers:
//		To change a configuration, read out the register, modify only the desired bits and write the new configuration. Blank fields may contain factory settings.
//		During power-up, configuration registers are reset to the permanently programmed value. Not programmed bits are zero.
#define		ZMCO			0x00	//R
#define		ZPOSA			0x01	//R/W/P
#define		ZPOSB			0x02	//R/W/P
#define		CONFA			0x07	//R/W/P
#define		CONFB			0x08	//R/W/P
#define		ABN				0x09	//R/W/P
#define		PUSHTHR			0x0A	//R/W/P

//Output Registers
#define		RAWANGLEA		0x0C	//R
#define		RAWANGLEB		0x0D	//R
#define		ANGLEA			0x0E	//R
#define		ANGLEB			0x0F	//R

//Status Registers
#define		STATUS			0x0B	//R
#define		AGC				0x1A	//R
#define		MAGNITUDEA		0x1B	//R
#define		MAGNITUDEB		0x1C	//R


//Burn Command
#define		BURN			0xFF	//W 

//Burn command options
#define		BURN_ANGLE		0x80
#define		BURN_SETTING	0x40

//configuration options
//	power mode
#define		PM_NOM			0x00
#define		PM_LPM1			0x01
#define		PM_LPM2			0x02
#define		PM_LPM3			0x03
//	Hysteresis
#define		HYST_OFF		0x00
#define		HYST_LSB1		0x01
#define		HYST_LSB2		0x02
#define		HYST_LSB3		0x03
//slow filter
#define		SF_16X			0x00
#define		SF_8X			0x01
#define		SF_4X			0x02
#define		SF_2X			0x03
//fast filter threshold
#define		FTH_SFO			0x00
#define		FTH_1			0x01
#define		FTH_2			0x02
#define		FTH_3			0x03
#define		FTH_4			0x04
#define		FTH_5			0x05
#define		FTH_6			0x06
#define		FTH_7			0x07
//Watchdog Timer
#define		WD_OFF			0x00
#define		WD_ON			0x01


//ABN Mapping
#define		ABN_8_61HZ		0x00
#define		ABN_16_122HZ	0x01
#define		ABN_32_244HZ	0x02
#define		ABN_64_488HZ	0x03
#define		ABN_128_976HZ	0x04
#define		ABN_256_1K9HZ	0x05
#define		ABN_512_3K9HZ	0x06
#define		ABN_1024_7K8HZ	0x07
#define		ABN_2048_15K6HZ	0x08

//STATUS options:
#define		STATUS_MD		0b00100000
#define		STATUS_ML		0b00010000
#define		STATUS_MH		0b00001000


bool		AS5601_Initialise(uint16_t config);						//Connect over I2C and configure the AS5601, calls the SetConfig with default values
bool		AS5601_SetRegister(uint8_t registerAddress);	//set the register for a read (only required when getting the angle so this can bet set one cand then read multiple times, other get commands set the correct register as part of the command)
/*
The CONF register supports customizing the AS5601 the following information is returned:
Bit position	Description
	1:0			Power Mode	00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
	3:2			Hysteresis	00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
	9:8			Slow Filter	00 = 16x (1); 01 = 8x; 10 = 4x; 11 = 2x
	12:10		Fast Filter Threshold 000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101 = 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
	13			Watchdog Timer 0 = OFF, 1 = ON (automatic entry into LPM3 low-power mode enabled)
*/
uint16_t	AS5601_GetConfig(void);

/*
POWER MODE
Three low-power modes are available. In a low-power mode, the fast filter is automatically disabled,
because there is no need for a fast settling time if the output refresh is as fast as the polling cycles.

Default 00 = NOM
*/
bool		AS5601_SetPowerMode(uint8_t);

/*
HYSTERESIS
To suppress spurious toggling of the output when the magnet is not moving, a 1 to 3 LSB hysteresis of the 12-bit resolution
can be enabled

Default 00 = OFF
*/
bool		AS5601_SetHysteresis(uint8_t);

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
bool		AS5601_SetSlowFilter(uint8_t);

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
bool		AS5601_SetFastFilter(uint8_t);

/*
WATCHDOG TIMER
The watchdog timer allows saving power by switching into LMP3 if the angle stays within
the watchdog threshold of 4 LSB for at least one minute,

Default OFF
*/
bool		AS5601_SetWatchdog(uint8_t);

/*
MH AGC minimum gain overflow, magnet too strong
ML AGC maximum gain overflow, magnet too weak
MD Magnet was detected
*/
uint8_t		AS5601_GetStatus(void);


bool		AS5601_DetectMagnet(void);		//Check if the magnet is detected (shortcut for GetStatus returning MD high)


uint16_t	AS5601_GetMagnitude(void);		//The MAGNITUDE register indicates the magnitude value of the internal CORDIC output.

/*
For the most robust performance, the gain value should be in the center of its range.
In 5V operation, the AGC range is 0-255 counts. The AGC range is reduced to 0-128 counts in 3.3V mode.
*/
uint8_t		AS5601_GetAGC(void);			


/*
The RAW ANGLE register contains the unmodified angle.
The zero adjusted and filtered output value is available in the ANGLE register.
*/
uint16_t	AS5601_GetRawAngle(void);
uint16_t	AS5601_GetAngle(void);

/*
It sets the current RAWANGLE to be "0"
*/
void		AS5601_SetCurrentZeroPosition();

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
void		AS5601_SetABN(uint8_t);

/*
Pass through the I2C connection state
*/
uint8_t AS5601_GetTWIStateInfo();

#endif //AS5601 included
