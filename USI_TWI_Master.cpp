/*****************************************************************************
*
*
* File              USI_TWI_Master.c compiled with gcc
* Date              Friday, 10/31/08		Boo!
* Updated by        jkl
*

* AppNote           : AVR310 - Using the USI module as a TWI Master
*
*		Extensively modified to provide complete I2C driver.
*	
*Notes: 
*		- T4_TWI and T2_TWI delays are modified to work with 1MHz default clock
*			and now use hard code values. They would need to change
*			for other clock rates. Refer to the Apps Note.
*
*	12/17/08	Added USI_TWI_Start_Memory_Read Routine		-jkl
*		Note msg buffer will have slave adrs ( with write bit set) and memory adrs;
*			length should be these two bytes plus the number of bytes to read.
****************************************************************************/

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
extern void _delay_loop_2(uint16_t __count);

#else

#include <util/delay_basic.h>

#endif

// Delay a smallish number of microseconds (max. 32767)
inline void _delay_us(uint16_t us)
{
	_delay_loop_2(us * 2);			// each iteration takes 4 clocks, so at 8MHz clock we need 2 iterations per microsecond
}

#include "USI_TWI_Master.h"

bool USI_TWI_Start_Transceiver_With_Data(uint8_t * array, uint8_t);
uint8_t USI_TWI_Master_Transfer(uint8_t);
bool USI_TWI_Master_Start();

struct USI_TWI_state_t
{
	uint8_t errorState;
	uint8_t addressMode         : 1;
	uint8_t masterWriteDataMode : 1;
	uint8_t memReadMode			: 1;
	uint8_t unused              : 5;
} USI_TWI_state;

/*---------------------------------------------------------------
 USI TWI single master initialization function
---------------------------------------------------------------*/
void USI_TWI_Master_Initialise( void )
{
	PORT_USI |= (1u << PIN_USI_SDA);           // Enable pullup on SDA, to set high as released state.
	PORT_USI |= (1u << PIN_USI_SCL);           // Enable pullup on SCL, to set high as released state.
  
	DDR_USI  |= (1u << PIN_USI_SCL);           // Enable SCL as output.
	DDR_USI  |= (1u << PIN_USI_SDA);           // Enable SDA as output.
  
	USIDR    =  0xFFu;                       // Preload dataregister with "released level" data.
	USICR    =  (0u << USISIE) | (0u << USIOIE) |                            // Disable Interrupts.
				(1u << USIWM1) | (0u << USIWM0) |                            // Set USI in Two-wire mode.
				(1u << USICS1) | (0u << USICS0) | (1u << USICLK) |                // Software stobe as counter clock source
				(0u << USITC);
	USISR    =  (1u << USISIF) | (1u << USIOIF) | (1u << USIPF) | (1u << USIDC) |      // Clear flags,
				(0x0u << USICNT0);                                     // and reset counter.
}

/*---------------------------------------------------------------
Use this function to get hold of the error message from the last transmission
---------------------------------------------------------------*/
uint8_t USI_TWI_Get_State_Info()
{
	return USI_TWI_state.errorState;                            // Return error state.
}

/*---------------------------------------------------------------
 USI Random (memory) Read function. This function sets up for call
 to USI_TWI_Start_Transceiver_With_Data which does the work.
 Doesn't matter if read/write bit is set or cleared, it'll be set
 correctly in this function.
 
 The msgSize is passed to USI_TWI_Start_Transceiver_With_Data.
 
 Success or error code is returned. Error codes are defined in 
 USI_TWI_Master.h
---------------------------------------------------------------*/
bool USI_TWI_Start_Random_Read(uint8_t * array msg, uint8_t msgSize)
{
	msg[0] &= ~(1u << TWI_READ_BIT);		// clear the read bit if it's set
	USI_TWI_state.errorState = 0;
	USI_TWI_state.memReadMode = true;
	return USI_TWI_Start_Transceiver_With_Data(msg, msgSize);
}

/*---------------------------------------------------------------
 USI Normal Read / Write Function
 Transmit and receive function. LSB of first byte in buffer 
 indicates if a read or write cycles is performed. If set a read
 operation is performed.

 Function generates (Repeated) Start Condition, sends address and
 R/W, Reads/Writes Data, and verifies/sends ACK.
 
 Success or error code is returned. Error codes are defined in 
 USI_TWI_Master.h
---------------------------------------------------------------*/
bool USI_TWI_Start_Read_Write(uint8_t * array msg, uint8_t msgSize)
{
	USI_TWI_state.errorState = 0;
	USI_TWI_state.memReadMode = false;
	return USI_TWI_Start_Transceiver_With_Data(msg, msgSize);
}

/*---------------------------------------------------------------
 USI Transmit and receive function. LSB of first byte in buffer 
 indicates if a read or write cycles is performed. If set a read
 operation is performed.

 Function generates (Repeated) Start Condition, sends address and
 R/W, Reads/Writes Data, and verifies/sends ACK.
 
 This function also handles Random Read function if the memReadMode
 bit is set. In that case, the function will:
 The address in memory will be the second
 byte and is written *without* sending a STOP. 
 Then the Read bit is set (lsb of first byte), the byte count is 
 adjusted (if needed), and the function function starts over by sending
 the slave address again and reading the data.
 
 Success or error code is returned. Error codes are defined in 
 USI_TWI_Master.h
---------------------------------------------------------------*/
bool USI_TWI_Start_Transceiver_With_Data(uint8_t * array msg, uint8_t msgSize)
{
	const uint8_t tempUSISR_8bit = (1u << USISIF) | (1u << USIOIF) | (1u << USIPF) | (1u << USIDC)|      // Prepare register value to: Clear flags, and
									(0x0u << USICNT0);                                     // set USI to shift 8 bits i.e. count 16 clock edges.
	const uint8_t tempUSISR_1bit = (1u << USISIF) | (1u << USIOIF) | (1u << USIPF) | (1u << USIDC) |      // Prepare register value to: Clear flags, and
									(0xEu << USICNT0); 									// set USI to shift 1 bit i.e. count 2 clock edges.
	//This clear must be done before calling this function so that memReadMode can be specified.
	//  USI_TWI_state.errorState = 0;				// Clears all mode bits also

	USI_TWI_state.addressMode = true;				// Always true for first byte

#ifdef PARAM_VERIFICATION
	if (msg > (unsigned char*)RAMEND)				// Test if address is outside SRAM space
	{
		USI_TWI_state.errorState = USI_TWI_DATA_OUT_OF_BOUND;
		return false;
	}
	if (msgSize <= 1)								// Test if the transmission buffer is empty
	{
		USI_TWI_state.errorState = USI_TWI_NO_DATA;
		return false;
	}
#endif

#ifdef NOISE_TESTING                                // Test if any unexpected conditions have arrived prior to this execution.
	if( USISR & (1u << USISIF) )
	{
		USI_TWI_state.errorState = USI_TWI_UE_START_CON;
		return false;
	}
	if( USISR & (1u << USIPF) )
	{
		USI_TWI_state.errorState = USI_TWI_UE_STOP_CON;
		return false;
	}
	if( USISR & (1u << USIDC) )
	{
		USI_TWI_state.errorState = USI_TWI_UE_DATA_COL;
		return false;
	}
#endif

	USI_TWI_state.masterWriteDataMode = ((msg[0] & (1u << TWI_READ_BIT)) == 0);                // The LSB in the address byte determines if is a masterRead or masterWrite operation.

	uint8_t * const array savedMsg = msg;
	const uint8_t savedMsgSize = msgSize; 

	if (!USI_TWI_Master_Start())
	{
		return false;                           // Send a START condition on the TWI bus.
	}

	// Write address and Read/Write data
	do
	{
		// If masterWrite cycle (or initial address transmission)
		if (USI_TWI_state.addressMode || USI_TWI_state.masterWriteDataMode)
		{
			// Write a byte
			PORT_USI &= ~(1u << PIN_USI_SCL);					// Pull SCL LOW.
			USIDR     = *(msg++);								// Setup data.
			USI_TWI_Master_Transfer(tempUSISR_8bit);			// Send 8 bits on bus.
      
			// Clock and verify (N)ACK from slave
			DDR_USI  &= ~(1u << PIN_USI_SDA);					// Enable SDA as input.
			if ((USI_TWI_Master_Transfer(tempUSISR_1bit) & (1u << TWI_NACK_BIT)) != 0) 
			{
				USI_TWI_state.errorState = (USI_TWI_state.addressMode) ? USI_TWI_NO_ACK_ON_ADDRESS : USI_TWI_NO_ACK_ON_DATA;
				return false;
			}
	  
			if (!USI_TWI_state.addressMode && USI_TWI_state.memReadMode)	// means memory start address has been written
			{
				msg = savedMsg;									// start at slave address again
				msg[0] |= (1u << TWI_READ_BIT);					// set the Read Bit on Slave address
				USI_TWI_state.errorState = 0;
				USI_TWI_state.addressMode = true;				// Now set up for the Read cycle
				msgSize = savedMsgSize;							// Set byte count correctly
				// Note that the length should be Slave addrs byte + # bytes to read + 1 (gets decremented below)
				if (!USI_TWI_Master_Start())
				{
					USI_TWI_state.errorState = USI_TWI_BAD_MEM_READ;
					return false;								// Send a START condition on the TWI bus.
				}
			}
			else
			{
				USI_TWI_state.addressMode = false;			// Only perform address transmission once.
			}
		}
		// Else masterRead cycle
		else
		{
			// Read a data byte
			DDR_USI &= ~(1u << PIN_USI_SDA);					// Enable SDA as input.
			*(msg++)  = USI_TWI_Master_Transfer(tempUSISR_8bit);

			// Prepare to generate ACK (or NACK in case of End Of Transmission)
			if (msgSize == 1)									// If transmission of last byte was performed.
			{
				USIDR = 0xFF;									// Load NACK to confirm End Of Transmission.
			}
			else
			{
				USIDR = 0x00;									// Load ACK. Set data register bit 7 (output for SDA) low.
			}
			USI_TWI_Master_Transfer(tempUSISR_1bit);			// Generate ACK/NACK.
		}
	} while (--msgSize != 0);									// Until all data sent/received.

	// Usually a stop condition is sent here, but TinyWireM needs to choose whether or not to send it
	return true;												// Transmission successfully completed
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be returned from the function.
---------------------------------------------------------------*/
uint8_t USI_TWI_Master_Transfer(uint8_t temp)
{
	USISR = temp;												// Set USISR according to temp.
																// Prepare clocking.
	temp = (0u << USISIE) | (0u << USIOIE) |					// Interrupts disabled
		   (1u << USIWM1) | (0u << USIWM0) |					// Set USI in Two-wire mode.
		   (1u << USICS1) | (0u << USICS0) | (1u << USICLK) |	// Software clock strobe as source.
		   (1u << USITC);										// Toggle Clock Port.
	do
	{ 
		_delay_us(T2_TWI);
		USICR = temp;											// Generate positve SCL edge.
		while((PIN_USI & (1u << PIN_USI_SCL)) == 0) { }			// Wait for SCL to go high.
		_delay_us(T4_TWI);
		USICR = temp;											// Generate negative SCL edge.
	} while((USISR & (1u << USIOIF)) == 0);						// Check for transfer complete.
  
	_delay_us(T2_TWI);
	temp  = USIDR;												// Read out data
	USIDR = 0xFF;												// Release SDA
	DDR_USI |= (1u << PIN_USI_SDA);								// Enable SDA as output

	return temp;												// Return the data from the USIDR
}	

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
bool USI_TWI_Master_Start()
{
	/* Release SCL to ensure that (repeated) Start can be performed */
	PORT_USI |= (1u << PIN_USI_SCL);							// Release SCL
	while((PORT_USI & (1u << PIN_USI_SCL)) == 0) { }			// Verify that SCL becomes high
	_delay_us(T2_TWI);

	/* Generate Start Condition */
	PORT_USI &= ~(1u << PIN_USI_SDA);							// Force SDA LOW
	_delay_us(T4_TWI);                         
	PORT_USI &= ~(1u << PIN_USI_SCL);							// Pull SCL LOW
	PORT_USI |= (1u << PIN_USI_SDA);							// Release SDA

#ifdef SIGNAL_VERIFY
	if ((USISR & (1u << USISIF)) == 0)
	{
		USI_TWI_state.errorState = USI_TWI_MISSING_START_CON;  
		return false;
	}
#endif
	return true;
}
/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
bool USI_TWI_Master_Stop()
{
	PORT_USI &= ~(1u << PIN_USI_SDA);							// Pull SDA low
	PORT_USI |= (1u << PIN_USI_SCL);							// Release SCL
	while((PIN_USI & (1u << PIN_USI_SCL)) == 0) { }				// Wait for SCL to go high  
	_delay_us(T4_TWI);
	PORT_USI |= (1u << PIN_USI_SDA);							// Release SDA
	_delay_us(T2_TWI);
  
#ifdef SIGNAL_VERIFY
	if ((USISR & (1u << USIPF)) == 0)
	{
		USI_TWI_state.errorState = USI_TWI_MISSING_STOP_CON;
		return false;
	}
#endif

	return true;
}

// End
