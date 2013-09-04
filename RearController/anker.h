#ifndef _ANKER_H_
#define _ANKER_H_

/*
 * Defines and typedefs
 */

#define MINIMUM_CURRENT_MA (200)
 
void ANKER_Init(uint8_t _msTick);
bool ANKER_MsTick(uint16_t currentmA);

#endif
