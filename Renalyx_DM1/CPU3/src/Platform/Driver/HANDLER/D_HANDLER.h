/*
 * D_HANDLER.h
 *
 * Created: 12/9/2014 2:54:52 PM
 *  Author: wid7
 */ 


#ifndef D_HANDLER_H_
#define D_HANDLER_H_

#include "asf.h"

typedef enum
{
	TICK_10MS,
	TICK_50MS,
	TICK_100MS,
	TICK_500MS,
	TICK_SEC
}tick_time;

typedef enum
{
	STATE_1,
	STATE_1_1
}acid_state;

typedef enum
{
	STATE_4,
	STATE_4_4
}bicar_state;

#endif /* D_HANDLER_H_ */