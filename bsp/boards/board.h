#ifndef __BOARD_H
#define __BOARD_H

/**
\addtogroup BSP
\{
\addtogroup board
\{

\brief Cross-platform declaration "board" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "board_info.h"
#include "toolchain_defs.h"

//=========================== define ==========================================

typedef enum {
   DO_NOT_KICK_SCHEDULER = 0,
   KICK_SCHEDULER        = 1,
} kick_scheduler_t;

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void board_init(void);
void board_sleep(void);
void board_reset(void);
void board_timer_init(void);
uint32_t board_timer_get(void);

/**
\}
\}
*/

#endif
