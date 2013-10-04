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
 * Local Application Includes
 */

#include "anker.h"

/*
 * Private Defines and Datatypes
 */

#define APPLICATION_TICK_MS (5)
#define HEARTBEAT_TICK_MS (500)

#define ANKER_PORT IO_PORTD
#define ANKER_PIN 6

#define HEARTBEAT_PORT PIND
#define HEARTBEAT_PIN 5

#define BRAKELIGHT_PORT IO_PORTC
#define BRAKELIGHT_PIN 5

#define LIGHT_MA_MAX (50)

#define TIMER_PWM_MAX (0x3FF)
#define LIGHTS_PWM_CHANNEL TMR_OCCHAN_B

enum application_state_enum
{
	STATE_CONFIGURING,
	STATE_RUNNING,
};
typedef enum application_state_enum APPLICATION_STATE_ENUM;

typedef bool (*SPI_HANDLER)(uint8_t input, uint8_t *reply);
typedef void (*APPLICATION_STATE_HANDLER)(void);

/*
 * Private Function Prototypes
 */

static void setupIO(void);
static void setupTimer(void);

static void handleSPI(void);
static bool brakeSensorSPIHandler(uint8_t input, uint8_t *reply);
static bool inputConfigSPIHandler(uint8_t input, uint8_t *reply);

static void updateLights(void);
static void updateAnker(void);

static void startBrakeSensorSPISeq(void);
static void startInputConfigSPISeq(void);

static uint8_t currentSettingmA(void);

// Application tick handlers
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
static TMR8_TICK_CONFIG appTick;
static TMR8_TICK_CONFIG hbTick;

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
static const uint8_t inputConfig[2] =
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
	
	ANKER_Init(APPLICATION_TICK_MS);
	
	SPI_SetMaster(LIBSPI_MSTRFREQ_FOSC64);

	startInputConfigSPISeq();
	
	sei();
	
	while (true)
	{
		if (SPI_TestAndClear(&spi))
		{
			handleSPI();
		}
		
		if (TMR8_Tick_TestAndClear(&appTick))
		{
			// Pass processing to the application handler for the current state
			stateHandlers[applicationState]();
		}

		if (TMR8_Tick_TestAndClear(&hbTick))
		{
			if (applicationState == STATE_RUNNING)
			{
				IO_Toggle(HEARTBEAT_PORT, HEARTBEAT_PIN);
			}
		}
	}
}

static void setupIO(void)
{
	IO_SetMode(ANKER_PORT, ANKER_PIN, IO_MODE_OUTPUT);
	IO_SetMode(HEARTBEAT_PORT, HEARTBEAT_PIN, IO_MODE_OUTPUT);
	IO_SetMode(BRAKELIGHT_PORT, BRAKELIGHT_PIN, IO_MODE_OUTPUT);
}

static void setupTimer(void)
{
	/* Allow Minimus to setup the microcontroller for itself */
	CLK_Init(F_CPU);
	CLK_SetPrescaler(clock_div_8);

	TMR8_Tick_Init(2, 0);

	appTick.reload = APPLICATION_TICK_MS;
	appTick.active = true;
	TMR8_Tick_AddTimerConfig(&appTick);

	hbTick.reload = HEARTBEAT_TICK_MS;
	hbTick.active = true;
	TMR8_Tick_AddTimerConfig(&hbTick);

	TMR16_SetSource(TMR_SRC_FCLK);
	TMR16_SetCountMode(TMR16_COUNTMODE_FASTPWM_10BIT);
	TMR16_SetOutputCompareMode(TMR_OUTPUTMODE_SET, LIGHTS_PWM_CHANNEL);
	TMR16_SetOutputCompareValue(0, LIGHTS_PWM_CHANNEL);
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
	fnSpiHandler = brakeSensorSPIHandler;
	SPI_SendByte(BP_PACKET_START, &spi);
}


static void startInputConfigSPISeq(void)
{
	// Send first byte for input config
	spiIsIdle = false;
	fnSpiHandler = inputConfigSPIHandler;
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
		brakes[1] = input;
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
		*reply = BP_IO_CONFIG;
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
		((uint16_t)(brakes[0] - minima[0]) * TIMER_PWM_MAX) / brakeRange[0],
		((uint16_t)(brakes[1] - minima[1]) * TIMER_PWM_MAX) / brakeRange[1]
	};
	
	//lightPWM = ((brakeSetting[0] * 3) / 4) + brakeSetting[1];
	incrementwithrollover(lightPWM, TIMER_PWM_MAX);
	
	TMR16_SetOutputCompareValue(lightPWM, LIGHTS_PWM_CHANNEL);
}

static void updateAnker(void)
{
	bool on = ANKER_MsTick( currentSettingmA() );

	IO_Control(ANKER_PORT, ANKER_PIN, on);
}

static uint8_t currentSettingmA(void)
{
	return (lightPWM * LIGHT_MA_MAX) / UINT16_MAX;
}
