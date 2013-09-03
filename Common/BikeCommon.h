#ifndef _BIKE_COMMON_H_
#define _BIKE_COMMON_H_

enum bike_protocol_enum
{
	BP_NONE,
	BP_PACKET_START,
	BP_PING,
	BP_INPUT_CONFIG,
	BP_READ_SENSORS,
	BP_OUTPUT_CTRL,
	BP_PACKET_END = 0xFE,
};
typedef enum bike_protocol_enum BIKE_PROTOCOL_ENUM;

enum bike_io_enum
{
	IO_PA0,
	IO_PA1,
	IO_PA2,
	IO_PA3,
	IO_PA4,
	IO_PB0,
	IO_PB2,
	IO_MAX,
};
typedef enum bike_io_enum BIKE_IO_ENUM;

/* Input Config Helpers */

#define BIKE_IO_INDEX_FROM_BYTE(byte) (((byte) & 0xF0) >> 4)
#define BIKE_IO_CONFIG_FROM_BYTE(byte) ((IO_CONFIG_ENUM)((byte) & 0x0F))

#define BIKE_IO_INDEX_TO_BYTE(idx) (((idx) << 4) & 0xF0)
#define BIKE_IO_CONFIG_TO_BYTE(idx) ((idx) & 0x0F)

/* Output Setting*/

#define BIKE_OUTPUT_SETTING_FROM_BYTE(byte) ((bool)(((byte) & 0x01) == 0x01))
#define BIKE_OUTPUT_SETTING_TO_BYTE(set) ((set) ? 0x01 : 0x00)

#endif
