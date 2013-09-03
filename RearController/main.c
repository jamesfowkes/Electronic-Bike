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
 * Other library includes
 */

#include "util_macros.h"

/*
 * Private Defines and Datatypes
 */

/*#define BRAKE_1_MIN 100
#define BRAKE_1_MAX 250
#define BRAKE_1_RANGE (BRAKE_1_MAX - BRAKE_1_MIN)

#define BRAKE_2_MIN 100
#define BRAKE_2_MAX 250
#define BRAKE_2_RANGE (BRAKE_2_MAX - BRAKE_2_MIN)
*/
#define APPLICATION_TICK_MS 5
/*
#define ANKER_PERIOD_S (20)
#define ANKER_PERIOD_MS (ANKER_PERIOD_S * 1000)
#define ANKER_PERIOD_TICKS (ANKER_PERIOD_MS / APPLICATION_TICK_MS)

#define ANKER_DUTY_CYCLE_PC (1)
#define ANKER_ON_MS ((ANKER_PERIOD_MS * ANKER_DUTY_CYCLE_PC) / 100)
#define ANKER_ON_TICKS (ANKER_ON_MS / APPLICATION_TICK_MS)
*/
#define ANKER_PORT PORTC
#define ANKER_PIN PIN4
/*
#define ANKER_CURRENT_SETTING_DUTY_10BIT 255

#define ANKER_PWM_CHANNEL TMR_OCCHAN_A
#define LIGHTS_PWM_CHANNEL TMR_OCCHAN_B*/

enum state_enum
{
	STATE_IDLE,
	STATE_STARTED,
	STATE_READ1_SENT,
	STATE_READ2_SENT,
	STATE_END_SENT,
	STATE_END
};
typedef enum state_enum STATE_ENUM;

/*
 * Private Variables
 */
static STATE_ENUM state = STATE_IDLE;
static uint16_t brakes[2] = {0, 0};

static SPI_DATA spi;
static TMR8_TICK_CONFIG tick;

/*
 * Private Function Prototypes
 */

static void setupIO(void);
static void setupTimer(void);

static void applicationTick(void);
static void handleSPI(void);
static void updateLights(void);
static void updateAnker(void);

int main(void)
{

	/* Disable watchdog: not required for this application */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	setupTimer();
	setupIO();
	
	sei();
	
	while (true)
	{
		/*if (SPI_TestAndClear(&spi))
		{
			handleSPI();
		}*/
		
		if (TMR8_Tick_TestAndClear(&tick))
		{
			applicationTick();
		}
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

	TMR16_SetOutputCompareMode(TMR_OUTPUTMODE_SET, ANKER_PWM_CHANNEL);
	TMR16_SetOutputCompareValue(ANKER_CURRENT_SETTING_DUTY_10BIT, ANKER_PWM_CHANNEL);

	TMR16_SetOutputCompareMode(TMR_OUTPUTMODE_SET, LIGHTS_PWM_CHANNEL);
	TMR16_SetOutputCompareValue(0, LIGHTS_PWM_CHANNEL);*/
}

static void applicationTick(void)
{
	updateLights();
	updateAnker();
}

static void handleSPI(void)
{
	uint8_t reply = spi.byte;
	
	STATE_ENUM thisState = state;
	++state;
	
	switch (thisState)
	{
	case STATE_IDLE:
		SPI_SendByte(BP_PACKET_START, &spi);
		// No reply - this is first byte of packet
		break;
	case STATE_STARTED:
		SPI_SendByte(BP_READ_BRAKE_SENSORS, &spi);
		// Nothing to do with idle reply (0xFF)
		break;
	case STATE_READ1_SENT:
		// Nothing to do with start reply (0x01)
		SPI_SendByte(BP_READ_BRAKE_SENSORS, &spi);
		break;
	case STATE_READ2_SENT:
		// Reply is first brake reading, and this is the last request
		brakes[0] = reply;
		SPI_SendByte(BP_PACKET_END, &spi);
		break;
	case STATE_END_SENT:
		// Reply is second brake reading, no more writes required
		brakes[1] = reply;
		break;
	case STATE_END:
		break; // Nothing to do and should never reach here
	}
	
	if (state == STATE_END)
	{
		state = STATE_IDLE;
	}
}

static void updateLights(void)
{
	/*brakes[0] = ((brakes[0] - BRAKE_1_MIN) * 100) / BRAKE_1_RANGE;
	brakes[1] = ((brakes[1] - BRAKE_2_MIN) * 100) / BRAKE_2_RANGE;
	
	brakes[0] = min(0xFFFF, brakes[0]);
	brakes[1] = min(0xFFFF, brakes[1]);
	
	uint16_t pwm = ((brakes[0] * 3) / 4) + brakes[1];
	
	TMR16_SetOutputCompareValue(pwm, LIGHTS_PWM_CHANNEL);*/
}

static void updateAnker(void)
{
	/*static uint16_t ticks = 0;
	
	if (ticks < ANKER_ON_TICKS)
	{
		IO_On(ANKER_PORT, ANKER_PIN);
	}
	else
	{
		IO_Off(ANKER_PORT, ANKER_PIN);
	}
	
	ticks = (ticks == ANKER_PERIOD_TICKS) ? 0 : ticks + 1;*/
	static bool on = false;

	if (on)
	{
		IO_Off(ANKER_PORT, ANKER_PIN);
	}
	else
	{
		IO_On(ANKER_PORT, ANKER_PIN);
	}

	on = !on;
}
