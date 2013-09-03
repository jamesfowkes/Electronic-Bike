/*
 * io_config.c
 *
 *  Allows dynamic setting of IO as input or output
 */
 
/*
 * Standard Library Includes
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * AVR Includes (Defines and Primitives)
 */

#include "avr/io.h"

/*
 * AVR Library Includes
 */

#include "lib_adc.h"
#include "lib_io.h"

/*
 * Local Application Includes
 */

#include "BikeCommon.h"
#include "io_config.h"

/*
 * Private Variables
 */
 
static const IO_PORT_ENUM ioPorts[IO_MAX] =
{
	IO_PORTA,
	IO_PORTA,
	IO_PORTA,
	IO_PORTA,
	IO_PORTA,
	IO_PORTB,
	IO_PORTB
};

static const uint8_t ioPins[IO_MAX] =
{
	PIN0,
	PIN1,
	PIN2,
	PIN3,
	PIN7,
	PIN0,
	PIN2
};

static LIB_ADC_CHANNEL_ENUM adcChannels[IO_MAX] =
{
	LIB_ADC_CH_0,
	LIB_ADC_CH_1,
	LIB_ADC_CH_2,
	LIB_ADC_CH_3,
	LIB_ADC_CH_4,
	LIB_ADC_CH_MAX,
	LIB_ADC_CH_MAX,
};

static IO_CONFIG_ENUM config[IO_MAX];

static uint8_t adcIndex = 0;

/*
 * Public Functions
 */

bool IOC_SetIO(uint8_t idx, bool set)
{
	bool success = true;
	if (config[idx] == OUTPUT)
	{
		IO_Control(ioPorts[idx], ioPins[idx], set);
	}
	else
	{
		success = false;
	}

	return success;
}

bool IOC_Read(uint8_t idx)
{
	bool success = false;
	if (config[idx] != OUTPUT)
	{
		success = IO_Read(ioPorts[idx], ioPins[idx]);
	}
	
	return success;
}

LIB_ADC_CHANNEL_ENUM IOC_GetNextADCChannel(void)
{
	do
	{
		adcIndex = (adcIndex == IO_MAX) ? 0 : adcIndex+1;
	}
	while (config[adcIndex] != INPUT_ADC);
	
	return adcChannels[adcIndex];
}

IO_PORT_ENUM IOC_GetPort(uint8_t idx)
{
	return ioPorts[idx];
}

uint8_t IOC_GetPin(uint8_t idx)
{
	return ioPins[idx];
}

LIB_ADC_CHANNEL_ENUM IOC_GetADCChannel(uint8_t idx)
{
	return adcChannels[idx];
}

bool IOC_NewInputConfig(BIKE_IO_ENUM idx, IO_CONFIG_ENUM newConfig)
{
	bool success = true;

	if ((idx < IO_MAX) && (newConfig <= OUTPUT))
	{
		config[idx] = newConfig;
		
		switch(newConfig)
		{
		case INPUT_ADC:
		case INPUT_NORMAL:
			IO_SetMode(ioPorts[idx], ioPins[idx], IO_MODE_INPUT);
			break;
		case INPUT_PULLUP:
			IO_SetMode(ioPorts[idx], ioPins[idx], IO_MODE_PULLUPINPUT);
			break;
		case OUTPUT:
			IO_SetMode(ioPorts[idx], ioPins[idx], IO_MODE_OUTPUT);
			break;
		}
	}
	else
	{
		success = false;
	}

	return success;

}
