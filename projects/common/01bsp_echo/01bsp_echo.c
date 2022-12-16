#include <stdio.h>

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"

//#include "nrf52840.h"
#include "happyserial.h"

//=========================== defines =========================================

//=========================== prototypes ======================================

void _happyserial_rx_cb(uint8_t* buf, uint8_t bufLen);

void     cb_uart_tx_done(void);
uint8_t  cb_uart_rx(void);

//=========================== variables =======================================

uint8_t stringToSend[]  = "+002 Ptest.24.00.12.-010\n";

typedef struct {
    uint32_t       dummy;
	volatile    uint8_t         uartDone;
    volatile    uint8_t         uartSendNow;
                uint8_t         uart_lastTxByteIndex;

} app_vars_t;

app_vars_t app_vars;

typedef struct {
    uint32_t       dummy;
} app_dbg_t;

app_dbg_t app_dbg;

//=========================== main ============================================

int mote_main(void) {
	//uint32_t i;
	
	memset(&app_vars,0,sizeof(app_vars_t));

	board_init();
	//uart_init();
    //uart_enableInterrupts();
    //uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);

    //app_vars.uartDone = 1;
    happyserial_init(_happyserial_rx_cb);
	
	uint8_t buf[] = {'a', 'b', 'c'};
	
	happyserial_tx(buf,sizeof(buf));
	

    // main loop
    while(1) {

        // wait for event
        //__SEV(); // set event
        //__WFE(); // wait for event
        //__WFE(); // wait for event
		//board_sleep();
		
		//happyserial_tx(0xFF,1);
		
		//uart_writeByte(0xFF);
		
		
		
    }
}

//=========================== interrupt handlers ==============================

void _happyserial_rx_cb(uint8_t* buf, uint8_t bufLen) {
	
    happyserial_tx(buf,bufLen);
	
}

uint8_t cb_uart_rx(void) {
    uint8_t byte;

    // toggle LED
    //leds_error_toggle();

    // read received byte
    byte = uart_readByte();

    // echo that byte over serial
    uart_writeByte(byte);

    return 0;
}

void cb_uart_tx_done(void) {
    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex<sizeof(stringToSend)) {
        uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
    } else {
        app_vars.uartDone = 1;
    }
}

