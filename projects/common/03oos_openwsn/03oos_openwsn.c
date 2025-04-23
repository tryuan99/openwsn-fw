/**
\brief This project runs the full OpenWSN stack.

*/

#include "config.h"
#include "board.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"

int mote_main(void) {
   
   // initialize
   board_init();
   scheduler_init();
   openstack_init();
   
   // start
   scheduler_start();
   return 0; // this line should never be reached
}

