/**
\brief Python-based emulation of the mote's power supply.

*/

#ifndef __SUPPLY_H
#define __SUPPLY_H

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

#include "openwsnmodule_obj.h"
typedef struct OpenMote OpenMote;

//=========================== prototypes ======================================

void supply_init(OpenMote* self);
void supply_on(OpenMote* self);
void supply_off(OpenMote* self);

#endif
