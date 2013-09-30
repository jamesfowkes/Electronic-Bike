/*
 * main.c
 *
 *  Startup file for sensor unit to be mounted
 *  at front of pedal bike for monitoring brake lever
 *  position.
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

#include "lib_adc.h"

#include "lib_clk.h"
#include "lib_tmr8_tick.h"

#include "BikeCommon.h"

/*
 * Local Application Includes
 */

#include "io_config.h"

/*
 * Private Defines and Datatypes
 */

#define MS_TICK 5

#define COMMS_ENABLE_PORT	IO_PORTB
#define COMMS_ENABLE_PIN	1

enum application_events_enum
{
	EVT_ADC_READ,
	EVT_SPI_BYTE,
	EVT_MAX_EVENTS
};
typedef enum application_events_enum APPLICATION_EVENTS_ENUM;

enum spi_state_enum
{
	SPI_STATE_IDLE,
	SPI_STATE_START,
	SPI_STATE_DATA,
};
typedef enum spi_state_enum SPI_STATE_ENUM;

/*
 * Function Prototypes
 */

static void applicationTick(void);

static uint8_t handleDataByte(void);

static uint8_t handleInputConfig(void);
static uint8_t getSensorReading(void);
static uint8_t newOutputs(void);

static void setupIO(void);
static void setupADC(void);
static void setupTimer(void);

static void enableComms(void);

static void spiHandler(void);
static void adcHandler(void);

/*
 * Private Variables
 */

 /* Library control structures */
static TMR8_TICK_CONFIG tick;
static ADC_CONTROL_ENUM adc;
static SPI_DATA spi;

/* ADC readings and control */
static uint8_t adcValues[IO_MAX];

static SPI_STATE_ENUM spiState = SPI_STATE_IDLE;
static BIKE_PROTOCOL_ENUM packetID = BP_NONE;

static bool on = false;

static uint8_t dataCounter = 0;

static uint16_t LEDMsTick = 0;

int main(void)
{

	/* Disable watchdog: not required for this application */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	setupIO();
	setupADC();
	setupTimer();

	enableComms();

	SPI_SetSlave(0xFF, &spi);
	
	sei();

	while (true)
	{

		if (SPI_TestAndClear(&spi))
		{
			spiHandler();
		}

		if (ADC_TestAndClear(&adc))
		{
			adcHandler();
		}

		if (TMR8_Tick_TestAndClear(&tick))
		{
			applicationTick();
		}

	}
}

/*
 * Private Functions
 */

static void setupIO(void)
{
	IOC_NewInputConfig(IO_PA0, INPUT_ADC);
	IOC_NewInputConfig(IO_PA1, INPUT_ADC);
	IOC_NewInputConfig(IO_PA2, INPUT_ADC);
	IOC_NewInputConfig(IO_PA3, INPUT_ADC);
	IOC_NewInputConfig(IO_PA4, INPUT_ADC);
	IOC_NewInputConfig(IO_PB0, OUTPUT);
	IOC_NewInputConfig(IO_PB2, INPUT_ADC);

	IO_SetMode(COMMS_ENABLE_PORT, COMMS_ENABLE_PIN, IO_MODE_OUTPUT);
}

static void setupADC(void)
{
	ADC_SelectPrescaler(LIB_ADC_PRESCALER_DIV64);
	ADC_SelectReference(LIB_ADC_REF_VCC);
	ADC_Enable(true);
	ADC_EnableInterrupts(true);

	adc.busy = false;
	adc.channel = LIB_ADC_CH_0;
	adc.conversionComplete = false;
}

static void setupTimer(void)
{
	CLK_Init(0);
	TMR8_Tick_Init();

	tick.reload = MS_TICK;
	tick.active = true;
	TMR8_Tick_AddTimerConfig(&tick);
}

static void enableComms(void)
{
	IO_Control(COMMS_ENABLE_PORT, COMMS_ENABLE_PIN, false);
}

static void applicationTick(void)
{

	LEDMsTick += MS_TICK;

	if (!adc.busy)
	{
		adc.channel = IOC_GetNextADCChannel();
		ADC_GetReading(&adc);
	}

	if (LEDMsTick == 500)
	{
		on ? IO_Off(PORTB, 0) : IO_On(PORTB, 0);
		on = !on;
		LEDMsTick = 0;
	}
}

static uint8_t handleInputConfig(void)
{
	// Upper nibble is IO index, lower nibble is new config type
	uint8_t io_index = BIKE_IO_INDEX_FROM_BYTE(spi.byte);
	IO_CONFIG_ENUM io_type = BIKE_IO_CONFIG_FROM_BYTE(spi.byte);

	uint8_t reply = 0x00;

	reply |= BIKE_IO_INDEX_TO_BYTE(io_index);
	reply |= BIKE_IO_CONFIG_TO_BYTE((uint8_t)io_type);

	return IOC_NewInputConfig(io_index, io_type) ? reply : 0x00;
}

static uint8_t getSensorReading(void)
{
	return adcValues[dataCounter % 2];
}

static uint8_t newOutputs(void)
{
	uint8_t io_index = BIKE_IO_INDEX_FROM_BYTE(spi.byte);
	bool set = BIKE_OUTPUT_SETTING_FROM_BYTE(spi.byte);

	uint8_t reply = 0x00;

	reply |= BIKE_IO_INDEX_TO_BYTE(io_index);
	reply |= BIKE_OUTPUT_SETTING_TO_BYTE(set);

	return IOC_SetIO(io_index, set) ? reply : 0x00;
}

static void adcHandler(void)
{
	adcValues[adc.channel] = (uint8_t)(adc.reading / 4);
}

static void spiHandler(void)
{

	uint8_t byte = spi.byte;
	uint8_t reply = BP_NONE;

	if ((byte == BP_PACKET_START) && (spiState == SPI_STATE_IDLE))
	{
		spiState = SPI_STATE_START;
		reply = BP_PACKET_START;
	}
	else if (spiState == SPI_STATE_START)
	{
		packetID = (BIKE_PROTOCOL_ENUM)byte;
		spiState = SPI_STATE_DATA;
		reply = byte;
		dataCounter = 0;
	}
	else if (spiState == SPI_STATE_DATA)
	{
		if (byte != BP_PACKET_END)
		{
			reply = handleDataByte();
			dataCounter++;
		}
		else
		{
			dataCounter = 0;
			reply = BP_PACKET_END;
			spiState = SPI_STATE_IDLE;
		}
	}

	SPI_SetReply(reply, &spi);

}

static uint8_t handleDataByte(void)
{
	uint8_t data = BP_NONE;

	switch (packetID)
	{
	case BP_PING:
		data = BP_PING;
		break;
	case BP_IO_CONFIG:
		data = handleInputConfig();
		break;
	case BP_READ_SENSORS:
		data = getSensorReading();
		break;
	case BP_OUTPUT_CTRL:
		data = newOutputs();
		break;
	case BP_NONE:
	case BP_PACKET_START:
	case BP_PACKET_END:
		break;
	}

	return data;

}
