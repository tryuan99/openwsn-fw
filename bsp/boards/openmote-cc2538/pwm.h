#ifndef __PWM_H
#define __PWM_H

/**
\addtogroup BSP
\{
\addtogroup pwm
\{

\brief A pwm module.

*/
#include "stdint.h"
#include "board.h"

// ========================== define ==========================================

// ========================== variable ========================================

// ========================== private =========================================

// ========================== protocol =========================================

/**
\brief Initialization sctimer.
*/
void pwm_init(void);
void pwm_enable(void);
void pwm_disable(void);
#endif