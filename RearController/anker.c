/*
 * Standard Library Includes
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * Defines and typedefs
 */

#define MINIMUM_TIME_MS (2000)
#define TIMING_PERIOD_MS (20000)
#define MEASUREMENT_PERIOD_MS (TIMING_PERIOD_MS - MINIMUM_TIME_MS)

/*
 * Local application includes
 */
 
#include "anker.h"

/*
 * Private function prototypes
 */
 
void reset(void);

/*
 * Private Variables
 */

static bool overMinimumThisPeriod = false;

static uint16_t overcurrentMsCount = 0;
static uint16_t periodMsCount = 0;
static uint8_t msTick = 0;

void ANKER_Init(uint8_t _msTick)
{
	msTick = _msTick;
	reset();
}

bool ANKER_MsTick(uint16_t currentmA)
{
	
	bool loadRequired = false;
	
	periodMsCount += msTick;
	
	if (periodMsCount < MEASUREMENT_PERIOD_MS)
	{
		if (currentmA > MINIMUM_CURRENT_MA)
		{
			// Count the number of ms over minimum current
			overcurrentMsCount += msTick;
			
			// Has there been this current for at least the minimum time?
			if (overcurrentMsCount > MINIMUM_TIME_MS)
			{
				overMinimumThisPeriod = true;
			}
		}
		else
		{
			// Reset after having dropped below minimum
			overcurrentMsCount = 0;
		}
	}
	else if (periodMsCount < TIMING_PERIOD_MS)
	{
		// Signal application to turn on extra load if required
		loadRequired = !overMinimumThisPeriod;
	}
	else
	{
		loadRequired = false;
		reset();
	}
	
	return loadRequired;
}

void reset(void)
{
	overMinimumThisPeriod = false;
	overcurrentMsCount = 0;
	periodMsCount = 0;
}
