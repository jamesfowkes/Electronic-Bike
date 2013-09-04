/*
 * main.c
 *
 *  Startup file for controller unit to
 *  vary brightness of rear "brake" light
 *  on pushbike based on brake level readings.
 *
 *  Powered by "Anker" battery which requires minimum load
 *  of 200mA for about 1s every 20s.
 */

/*
 * Standard Library Includes
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

/*
 * AVR Includes (Defines and Primitives)
 */
#include "avr/io.h"
#include "avr/wdt.h"
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

/*
 * AVR Library Includes
 */

#include "lib_spi_common.h"
#include "lib_spi.h"
#include "lib_io.h"

#include "lib_clk.h"
#include "lib_tmr8_tick.h"
#include "lib_tmr16.h"

#include "BikeCommon.h"

/*
 * Utility Library includes
 */

#include "util_macros.h"

/*
 * Generic Library Includes
 */

#include "heartbeat.h"

/*
 * Local Application Includes
 */

#include "anker.h"

/*
 * Private Defines and Datatypes
 */

#define APPLICATION_TICK_MS (5)
#define HEARTBEAT_TIME_MS (500)

#define ANKER_PORT PORTC
#define ANKER_PIN PIN4

#define LIGHT_MA_MAX (50)

/*
#define LIGHTS_PWM_CHANNEL TMR_OCCHAN_B
*/

enum application_state_enum
{
	STATE_CONFIGURING,
	STATE_RUNNING,
};
typedef enum application_state_enum APPLICATION_STATE_ENUM;

typedef bool (*SPI_HANDLER)(uint8_t input, uint8_t *reply);
typedef void (*APPLICATION_STATE_HANDLER)*void);

/*
 * Private Function Prototypes
 */

static void setupIO(void);
static void setupTimer(void);

static void handleSPI(void);
static void updateLights(void);
static void updateAnker(void);

static void startBrakeSensorSPISeq(void);
static void startInputConfigSPISeq(void);

// Application tick handlers
static void applicationGlobalTick(void);
static void configHandler(void);
static void runningHandler(void);

/*
 * Private Variables
 */
static APPLICATION_STATE_ENUM applicationState = STATE_CONFIGURING;

static uint8_t minima[2] = {128, 128};
static uint8_t maxima[2] = {-127, -127};

static uint8_t brakes[2] = {0, 0};

static SPI_DATA spi;
static TMR8_TICK_CONFIG tick;

static uint16_t lightPWM = 0;

static SPI_HANDLER fnSpiHandler = NULL;
static uint8_t spiByteCount = 0;
static bool spiIsIdle = true;

static APPLICATION_STATE_HANDLER stateHandlers[] = 
{
	configHandler, /* STATE_CONFIGURING */
	runningHandler /* STATE_RUNNING */
};

// Configuration of forward sensor inputs
static const inputConfig[2] = 
{
	BIKE_IO_INDEX_TO_BYTE(IO_PA0) | BIKE_IO_CONFIG_TO_BYTE(INPUT_ADC),
	BIKE_IO_INDEX_TO_BYTE(IO_PA1) | BIKE_IO_CONFIG_TO_BYTE(INPUT_ADC)
};

int main(void)
{

	/* Disable watchdog: not required for this application */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	setupTimer();
	setupIO();
	
	Heartbeat_Init(APPLICATION_TICK_MS, HEARTBEAT_TIME_MS);
	
	ANKER_Init(APPLICATION_TICK_MS);
	
	startInputConfigSPISeq();
	
	sei();
	
	while (true)
	{
		/*if (SPI_TestAndClear(&spi))
		{
			handleSPI();
		}*/
		
		if (TMR8_Tick_TestAndClear(&tick))
		{
			// Pass processing to the application handler for the current state
			stateHandlers[applicationState]();
			
			// Do any global application level stuff
			applicationGlobalTick();
		}
	}
}

static void applicationGlobalTick(void)
{
	if ( Heartbeat_Tick() )
	{
		IO_Toggle(HEARTBEAT_PORT, HEARTBEAT_PIN);
	}
	
}
static void setupIO(void)
{
	IO_SetMode(ANKER_PORT, ANKER_PIN, IO_MODE_OUTPUT);
}

static void setupTimer(void)
{
	/* Allow Minimus to setup the microcontroller for itself */
	CLK_Init(F_CPU);
	CLK_SetPrescaler(clock_div_1);

	TMR8_Tick_Init();

	tick.reload = APPLICATION_TICK_MS;
	tick.active = true;
	TMR8_Tick_AddTimerConfig(&tick);

	/*TMR16_SetCountMode(TMR16_COUNTMODE_FASTPWM_10BIT);
	TMR16_SetOutputCompareMode(TMR_OUTPUTMODE_SET, LIGHTS_PWM_CHANNEL);
	TMR16_SetOutputCompareValue(0, LIGHTS_PWM_CHANNEL);*/
}

static void configHandler(void)
{
	if (spiIsIdle)
	{
		applicationState = STATE_RUNNING;
	}
}

static void runningHandler(void)
{
	if (spiIsIdle)
	{
		startBrakeSensorSPISeq();
	}
	updateAnker();
	updateLights();
}

static void startBrakeSensorSPISeq(void)
{
	// Send first byte for brake position sense
	spiIsIdle = false;
	fnSpiHandler = inputConfigHandler;
	SPI_SendByte(BP_PACKET_START, &spi);
}


static void startInputConfigSPISeq(void)
{
	// Send first byte for input config
	spiIsIdle = false;
	fnSpiHandler = brakeSensorSPIHandler;
	SPI_SendByte(BP_PACKET_START, &spi);
}

static void handleSPI(void)
{
	bool sendReply = false;
	uint8_t reply = 0x00;
	
	sendReply = fnSpiHandler(spi.byte, &reply);
	
	if (sendReply)
	{
		SPI_SendByte(reply, &spi);
		spiByteCount++;
	}
	else
	{
		spiIsIdle = true;
		spiByteCount = 0;
	}
}

static bool brakeSensorSPIHandler(uint8_t input, uint8_t *reply)
{
	bool sendReply = true;
	
	switch(spiByteCount)
	{
	case 0:
		*reply = BP_READ_SENSORS;
		// Nothing to do with idle reply (0xFF)
		break;
	case 1:
		// Nothing to do with start reply (0x01)
		*reply = BP_READ_SENSORS;
		break;
	case 2:
		// Reply is first brake reading, and this is the last request
		brakes[0] = input;
		*reply = BP_PACKET_END;
		break;
	case 3:
		// Reply is second brake reading, no more writes required
		brakes[1] = reply;
		sendReply = false;
		break;
	default:
		sendReply = false;
		break; // Nothing to do and should never reach here
	}
	
	return sendReply;
}

static bool inputConfigSPIHandler(uint8_t input, uint8_t *reply)
{
	bool sendReply = true;
	
	switch(spiByteCount)
	{
	case 0:
		*reply = BP_INPUT_CONFIG;
		break;
	case 1:
		*reply = inputConfig[0];
		break;
	case 2:
		*reply = inputConfig[1];
		break;
	case 3:
		*reply = BP_PACKET_END;
		break;
	case 4:
		sendReply = false;
		break;
	default:
		sendReply = false;
		break;
	}
	
	return sendReply;
}

static void updateLights(void)
{
	// Get new min and max values
	minima[0] = min(brakes[0], minima[0]);
	minima[1] = min(brakes[1], minima[1]);
	maxima[0] = min(brakes[0], maxima[0]);
	maxima[1] = min(brakes[1], maxima[1]);
	
	// Calculate the new braking range
	uint8_t brakeRange[2] = {
		maxima[0] - minima[0],
		maxima[1] - minima[1]
	};
	
	// Get the current brake setting and convert from 8- to 16-bit for PWM
	uint16_t brakeSetting[2] = {
		((uint16_t)(brakes[0] - minima[0]) * UINT16_MAX) / brakeRange[0],
		((uint16_t)(brakes[1] - minima[1]) * UINT16_MAX) / brakeRange[1]
	};
	
	lightPWM = ((brakeSetting[0] * 3) / 4) + brakeSetting[1];
	
	TMR16_SetOutputCompareValue(lightPWM, LIGHTS_PWM_CHANNEL);*/
}

static void updateAnker(void)
{
	bool on = ANKER_MsTick( currentSettingmA() );

	if (on)
	{
		IO_Off(ANKER_PORT, ANKER_PIN);
	}
	else
	{
		IO_On(ANKER_PORT, ANKER_PIN);
	}
}

static uint8_t currentSettingmA(void)
{
	return (lightPWM * LIGHT_MA_MAX) / UINT16_MAX;
}
