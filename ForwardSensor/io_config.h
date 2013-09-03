#ifndef _IO_CONFIG_H_
#define _IO_CONFIG_H_

/*
 * Defines and typedefs
 */
 
enum io_config_enum
{
	INPUT_NORMAL,
	INPUT_PULLUP,
	INPUT_ADC,
	OUTPUT
};
typedef enum io_config_enum IO_CONFIG_ENUM;

/*
 * Public Function Prototypes
 */

bool					IOC_SetIO(uint8_t idx, bool set);
bool					IOC_Read(uint8_t idx);

LIB_ADC_CHANNEL_ENUM	IOC_GetNextADCChannel(void);
IO_PORT_ENUM			IOC_GetPort(uint8_t idx);
uint8_t					IOC_GetPin(uint8_t idx);
LIB_ADC_CHANNEL_ENUM	IOC_GetADCChannel(uint8_t idx);

bool					IOC_NewInputConfig(BIKE_IO_ENUM idx, IO_CONFIG_ENUM newConfig);

#endif
