/*
 * FilamentSensor.cpp
 *
 * Created: 20/04/2017 21:41:39
 * Authors: tony@think3dprint3d.com and dcrocker@eschertech.com
 
 Library and example code from jkl http://www.cs.cmu.edu/~dst/ARTSI/Create/PC%20Comm/
 and from the arduino version more focused on the ATTiny85 http://playground.arduino.cc/Code/USIi2c
 
 2017-08 12 Changed bit rate to 2000bps

 */ 

#include "ecv.h"

#ifdef __ECV__
#define __attribute__(_x)
#define __volatile__
#pragma ECV noverifyincludefiles
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/fuse.h>

#ifdef __ECV__
#pragma ECV verifyincludefiles
#undef cli
#undef sei
extern void cli();
extern void sei();
#endif

#include "AS5601.h"

constexpr uint8_t FirmwareVersion = 3;

#define DOUBLE_SPEED	(0)									// set nonzero for 2000bps, zero for 1000bps

#define BITVAL(_x) static_cast<uint8_t>(1u << (_x))

const uint32_t F_CPU = 8000000UL;

// Pin allocations on the ATTINY44A implementation
#define PIN_SWITCH	PINA
const unsigned int PinSwitchBitNum = 2;						// optional filament present switch with external pullup resistor

#define PORT_LED	PORTB
#define DDR_LED		DDRB
const unsigned int PortLedRedBitNum = 0;
const unsigned int PortLedGreenBitNum = 1;

#define PORT_OUT	PORTA
#define DDR_OUT		DDRA
const unsigned int PortOutBitNum = 7;

#define PIN_PROG	PINB
const unsigned int PinProgBitNum = 2;						// programming push button

const uint8_t PortAPullupBitMask = BITVAL(3);				// PA3 is unused
const uint8_t PortBPullupBitMask = BITVAL(PinProgBitNum);	// need to enable pullup resistor on programming button pin
const uint8_t PortAOutputMask = BITVAL(PortOutBitNum);
const uint8_t PortBOutputMask = BITVAL(PortLedRedBitNum) | BITVAL(PortLedGreenBitNum);	// enable LED pins as outputs

const uint8_t LedRed = BITVAL(PortLedRedBitNum);
const uint8_t LedGreen = BITVAL(PortLedGreenBitNum);

#if DOUBLE_SPEED
const uint16_t TicksPerSecond = 2000;						// this must be the same as the desired bit rate
#else
const uint16_t TicksPerSecond = 1000;						// this must be the same as the desired bit rate
#endif

// Error codes (expressed as number of blinks
const uint8_t FLASHES_OK = 3;
const uint8_t FLASHES_ERR_I2C = 4;
const uint8_t FLASHES_ERR_I2C_STATE = 5;
const uint8_t FLASHES_ERR_NOMAG = 6;
const uint8_t FLASHES_ERR_TOO_WEAK = 7;
const uint8_t FLASHES_ERR_TOO_STRONG = 8;

const uint16_t AS5601Config = ConfFTH_18LSB | ConfSF_2X | ConfHYST_LSB3 | ConfPM_NOM;

const unsigned int BitsToDiscard = 2;						// discard the two LSB of the angle

#if DOUBLE_SPEED
const uint16_t MinOutputIntervalTicks = TicksPerSecond/50;	// send the angle 50 times per second while it is changing
#else
const uint16_t MinOutputIntervalTicks = TicksPerSecond/25;	// send the angle 25 times per second while it is changing
#endif

const uint16_t MaxOutputIntervalTicks = TicksPerSecond/2;	// send the angle at least every half second

const uint16_t kickFrequency = 10;							// how often we kick the watchdog
const uint16_t kickIntervalTicks = TicksPerSecond/kickFrequency;

// Version 1 sensor:
//  Data word:			0S00 00pp pppppppp		S = switch open, pppppppppp = 10-bit filament position
//  Error word:			1000 0000 00000000
//
// Version 2 sensor (this firmware):
//  Data word:			P00S 10pp pppppppp		S = switch open, ppppppppppp = 10-bit filament position
//  Error word:			P010 0000 0000eeee		eeee = error code
//	Version word:		P110 0000 vvvvvvvv		vvvvvvvv = sensor/firmware version, at least 2
//
// Version 3 firmware:
//  Data word:			P00S 10pp pppppppp		S = switch open, ppppppppppp = 10-bit filament position
//  Error word:			P010 0000 0000eeee		eeee = error code
//	Version word:		P110 0000 vvvvvvvv		vvvvvvvv = sensor/firmware version, at least 2
//  Magnitude word		P110 0010 mmmmmmmm		mmmmmmmm = highest 8 bits of magnitude (cf. brightness of laser sensor)
//  AGC word			P110 0011 aaaaaaaa		aaaaaaaa = AGC setting (cf. shutter of laser sensor)

const uint16_t ParityBit = 0x8000u;							// adjusted so that there is an even number of bits
const uint16_t SwitchOpenBit = 0x1000u;

const uint16_t PositionBits = 0x0800u;						// set in words carrying position data
const uint16_t ErrorBits = 0x2000u;							// set if the sensor failed to initialize or self-test
const uint16_t VersionBits = 0x6000u;						// set in words containing version information
const uint16_t MagnitudeBits = 0x6200u;						// set in words containing magnitude information
const uint16_t AgcBits = 0x6300u;							// set in words containing AGC information

const uint16_t ErrorBlinkTicks = TicksPerSecond/4;			// fast error blinks
const uint16_t OkBlinkTicks = TicksPerSecond/2;				// slow OK blinks

#ifndef __ECV__
FUSES = {0xE2u, 0xDFu, 0xFFu};		// 8MHz RC clock
#endif

uint16_t lastAngle = 0;
volatile uint16_t tickCounter = 0;
uint16_t lastKickTicks = 0;
uint16_t lastOutTicks = 0;

// Forward declarations
void blink(uint8_t count, uint8_t leds, uint16_t delayOn, uint16_t delayOff);
void SendWord(uint16_t data, uint8_t ledMask);

// Get a 16-bit volatile value from outside the ISR. As it's more than 8 bits long, we need to disable interrupts while fetching it.
inline uint16_t GetVolatileWord(volatile uint16_t& val)
writes(volatile)
{
	cli();
	const uint16_t locVal = val;
	sei();
	return locVal;
}

// Check whether we need to kick the watchdog
void CheckWatchdog()
writes(lastKickTicks; volatile)
{
	if (GetVolatileWord(tickCounter) - lastKickTicks >= kickIntervalTicks)
	{
#ifndef __ECV__
		wdt_reset();											// kick the watchdog
#endif
		lastKickTicks += kickIntervalTicks;
	}
}

// Delay for a specified number of ticks, kicking the watchdog as needed
void DelayTicks(uint16_t ticks)
writes(lastKickTicks; volatile)
{
	const uint16_t startTicks = GetVolatileWord(tickCounter);
	for (;;)
	{
		CheckWatchdog();
		if (GetVolatileWord(tickCounter) - startTicks >= ticks)
		{
			break;
		}
	}
}

// Report an error. The output must have been in the low state for at least 10 ticks before calling this.
void ReportError(uint8_t errorNum)
{
	SendWord(ErrorBits | errorNum, 0);							// send error to the Duet without lighting the LED
	blink(errorNum, LedRed, ErrorBlinkTicks, ErrorBlinkTicks);	// short error blinks, also leaves the output in the low state for ErrorBlinkTicks
}

uint8_t StatusToErrorCode(uint8_t status)
{
	return  ((status & StatusMH) != 0) ? FLASHES_ERR_TOO_STRONG
			: ((status & StatusMD) == 0) ? FLASHES_ERR_NOMAG
				: ((status & StatusML) != 0) ? FLASHES_ERR_TOO_WEAK
					: FLASHES_OK;
}

int main(void)
{  
	// Set up the I/O ports
	PORTA = PortAPullupBitMask;
	DDRA = PortAOutputMask;
	PORTB = PortBPullupBitMask;
	DDRB = PortBOutputMask;

	// Setup the timer to generate the tick interrupt
#if DOUBLE_SPEED
	// For a tick rate of 2000Hz we need a total divisor of 4000, for example 250 * 16 or 125 * 32.
	// Timer/counter 0 only offers prescalers of 1, 8, 64, 256 and 1024. But we can get 16 by using dual slope mode and prescaler 8.
	TCCR0A = BITVAL(WGM00);									// phase correct PWM mode, count up to OCR0A then count down
	TCCR0B = BITVAL(WGM02) | BITVAL(CS01);					// prescaler 8
	OCR0A = F_CPU/(16 * TicksPerSecond) - 1;				// set the period to the bit time
#else
	// For a tick rate of 1000 we need a total divisor of 8000, for example 125 * 64
	TCCR0A = BITVAL(WGM01);									// CTC mode, count up to OCR0A then start again from zero
	TCCR0B = BITVAL(CS01) | BITVAL(CS00);					// prescaler 64
	OCR0A = F_CPU/(64 * TicksPerSecond) - 1;				// set the period to the bit time
#endif
	TCNT0 = 0;
	TIMSK0 |= BITVAL(OCIE0A);								// enable timer compare match interrupt

#ifndef __ECV__												// eCv++ doesn't understand gcc assembler syntax
	wdt_enable(WDTO_500MS);									// enable the watchdog
#endif

	sei();

	for (;;)
	{
		PORT_OUT &= ~BITVAL(PortOutBitNum);					// ensure output is in default low state
		DelayTicks(2 * TicksPerSecond);						// allow the power voltage to stabilise, or give a break from flashing the previous error
		if (!AS5601_Initialise(AS5601Config))
		{
			ReportError(FLASHES_ERR_I2C);
		}
		else if (AS5601_GetTWIStateInfo() != 0)
		{
			ReportError(FLASHES_ERR_I2C_STATE);
		}
		else
		{
			// Need at least a 1ms delay after writing the configuration register according to the datasheet. In fact, 1ms doesn't seem to be long enough, so we allow 20ms.
			DelayTicks(TicksPerSecond/50);
			const uint8_t status = AS5601_GetStatus();
			if (status == StatusMD)
			{
				break;
			}
			ReportError(StatusToErrorCode(status));
		}
	}

	blink(FLASHES_OK, LedGreen, OkBlinkTicks, OkBlinkTicks);	// blink 3 times after successful initialisation

	AS5601_SetABN(ABN_2048_15K6HZ);								// set to 2048 resolution
	AS5601_SetCurrentZeroPosition();							// set the current angle to zero
	PORT_OUT &= ~BITVAL(PortOutBitNum);							// ensure output is in default low state

	// Main send loop. The algorithm is:
	// - Don't send anything unless the minimum interval has elapsed since we last sent something
	// - If the status is good, and the position has changed, send the position (i.e. angle and switch)
	// - If the status has changed from bad to good, send the position. RRF will clear its error status when it received a position message.
	// - If the status has changed from good to bad, set the info state to 0 so that we send the status
	// - Else if the info state isn't 0, send the next info word and advance the info state
	// - Else do nothing unless the maximum interval has elapsed
	// - Else if we sent an info word last time and the status is good, send the angle and switch
	// - Else send the status
	bool sentPosition = false;
	uint8_t infoSendState = 0;
	bool lastStatusGood = false;
	for (;;)
	{
		CheckWatchdog();

		const uint16_t now = GetVolatileWord(tickCounter);
		const uint16_t diff = now - lastOutTicks;
		if (diff >= MinOutputIntervalTicks)
		{
			bool sendAngle = false;
			bool sendInfo = false;

			const uint8_t status = AS5601_GetStatus();
			const uint8_t statusGood = (status == StatusMD);
			if (statusGood)
			{
				uint16_t currentAngle = AS5601_GetAngle() >> BitsToDiscard;
				if ((PIN_SWITCH & BITVAL(PinSwitchBitNum)) != 0)
				{
					currentAngle |= SwitchOpenBit;				// send the switch bit too
				}
				if (currentAngle != lastAngle || !lastStatusGood)
				{
					lastAngle = currentAngle;
					sendAngle = true;
				}
			}
			else if (lastStatusGood)							// if status has gone from OK to not OK
			{
				infoSendState = 0;
				sendInfo = true;
			}

			lastStatusGood = statusGood;

			if (!sendAngle && !sendInfo)
			{
				if (infoSendState != 0)
				{
					sendInfo = true;
				}
				else if (diff >= MaxOutputIntervalTicks)
				{
					if (!sentPosition && statusGood)
					{
						sendAngle = true;
					}
					else
					{
						sendInfo = true;
					}
				}
			}

			if (sendAngle)
			{
				lastOutTicks = now;
				SendWord(PositionBits | lastAngle, LedGreen);
				sentPosition = true;
			}
			else if (sendInfo)
			{
				switch (infoSendState)
				{
				case 0:
					{
						uint8_t errorCode = StatusToErrorCode(status);
						if (errorCode != FLASHES_OK)
						{
							SendWord(ErrorBits | errorCode, LedRed);
							break;
						}
					}
					++infoSendState;
					// no break
				case 1:
					SendWord(VersionBits | FirmwareVersion, LedRed);
					break;

				case 2:
					SendWord(AgcBits | AS5601_GetAGC(), LedRed);
					break;

				case 3:
					SendWord(MagnitudeBits | ((AS5601_GetMagnitude() >> 4) & 0x00FF), LedRed);
					break;
				}
				lastOutTicks = now;
				sentPosition = false;
				++infoSendState;
				if (infoSendState == 4)
				{
					infoSendState = 0;
				}
			}
		}
	}

#ifdef __ECV__
	return 0;
#endif
}

// Timer ISR for setting output flag
#ifdef __ECV__
void tickIsr()
#else
ISR(TIM0_COMPA_vect)
#endif
{
	tickCounter++;
}

// Wait for the next tick. This does not call the watchdog, so don't call this too many times without making a call to checkWatchdog.
inline void WaitForNextTick()
{
	const volatile uint8_t * const tickLsb = reinterpret_cast<const volatile uint8_t *>(&tickCounter);
	const uint8_t initialCount = *tickLsb;
	while (*tickLsb == initialCount) { }
}

inline void SendZeroBit()
{
	PORT_OUT &= ~BITVAL(PortOutBitNum);
}

inline void SendOneBit()
{
	PORT_OUT |= BITVAL(PortOutBitNum);
}

// Send a 16-bit word
void SendWord(uint16_t data, uint8_t ledMask)
{
	PORT_LED |= ledMask;							// turn on the LED because it is separate from the output pin

	// Calculate the parity bit
	uint8_t data8 = (uint8_t)((data >> 8) ^ data);
	data8 ^= (data8 >> 4);
	data8 ^= (data8 >> 2);
	data8 ^= (data8 >> 1);
	if (data8 & 1)
	{
		data ^= ParityBit;
	}

	WaitForNextTick();								// this one will be a full bit length
	SendOneBit();									// set output high for the start bit
	WaitForNextTick();
	SendZeroBit();									// return output to low for the end of the start bit

	// Send 4 nibbles + stuffing bits
	for (uint8_t nibble = 0; nibble < 4; ++nibble)
	{
		bool b;
		for (uint8_t i = 0; i < 4; ++i)
		{
			b = ((data & 0x8000u) != 0);
			WaitForNextTick();
			if (b)
			{
				SendOneBit();
			}
			else
			{
				SendZeroBit();
			}
			data <<= 1;
		}

		// Send the stuffing bit, which is the opposite of the last bit
		WaitForNextTick();
		if (b)
		{
			SendZeroBit();
		}
		else
		{
			SendOneBit();
		}

		CheckWatchdog();
	}

	// Stop bit
	WaitForNextTick();
	SendZeroBit();									// return output to default low state
	WaitForNextTick();
	WaitForNextTick();

	PORT_LED &= ~ledMask;
}

/*------------------------------------------------------------------------
**  blinkCustom - function to blink LED for count passed in
**		Assumes that leds are all on the same port. 
**     Custom on and off times can be set in ~0.1s increments
** ---------------------------------------------------------------------*/
void blink(uint8_t count, uint8_t ledMask, uint16_t delayOn, uint16_t delayOff)
{
	while (count != 0)
	{
		PORT_LED |= ledMask;
		DelayTicks(delayOn);
		PORT_LED &= ~ledMask;
		DelayTicks(delayOff);
		count--;
	}
}

// End
