/**
\brief CoAP 6top application

*/

#ifndef __C6T_H
#define __C6T_H

/**
\addtogroup AppCoAP
\{
\addtogroup c6t
\{
*/

#include "config.h"
#include "opendefs.h"
#include "coap.h"
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    coap_resource_desc_t desc;
} c6t_vars_t;

//=========================== prototypes ======================================

void c6t_init(void);

/**
\}
\}
*/

#endif
