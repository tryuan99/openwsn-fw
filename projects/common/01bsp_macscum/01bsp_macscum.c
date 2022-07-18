/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

After loading this program, your board will switch on its radio on frequency
CHANNEL.

While receiving a packet (i.e. from the start of frame event to the end of
frame event), it will turn on its sync LED.

Every TIMER_PERIOD, it will also send a packet containing LENGTH_PACKET bytes
set to ID. While sending a packet (i.e. from the start of frame event to the
end of frame event), it will turn on its error LED.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include <stdio.h>

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"

//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC  ///< maximum length is 127 bytes
//#define LEN_PKT_TO_SEND 9+LENGTH_CRC
#define LEN_PKT_TO_SEND 9+LENGTH_CRC
#define CHANNEL         17             ///< 11=2.405GHz
#define TIMER_PERIOD    (0x7fff)  	   ///< 0xffff = 2s@32kHz
#define ID              0x99           ///< byte sent in the packets

#define MOTE_ADDRESS	0x1234
#define JOIN_REQ_PKT_LEN	14+LENGTH_CRC

#define PACKET_TYPE__JOIN	0x0044

uint8_t stringToSend[]  = "+002 Ptest.24.00.12.-010\n";

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
    volatile    uint8_t         uartDone;
    volatile    uint8_t         uartSendNow;
                uint8_t         uart_lastTxByteIndex;

                uint8_t         flags;
                app_state_t     state;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
				
				uint8_t			target_address_msb;
				uint8_t			target_address_lsb
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

void     cb_uart_tx_done(void);
uint8_t  cb_uart_rx(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint8_t i;
	uint16_t j;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    // setup UART
    uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    uart_enableInterrupts();

    app_vars.uartDone = 1;

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);
    for (i=0;i<app_vars.packet_len;i++) {
        app_vars.packet[i] = ID;
    }

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);

    // switch in RX by default
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;

    // start by a transmit
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {

        // sleep while waiting for at least one of the flags to be set
        while (app_vars.flags==0x00) {
            board_sleep();
        }

        // handle and clear every flag
        while (app_vars.flags) {


            //==== APP_FLAG_START_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_START_FRAME) {
                // start of frame

                switch (app_vars.state) {
                    case APP_STATE_RX:
                        // started receiving a packet

                        // led
                        leds_error_on();
                        break;
                    case APP_STATE_TX:
                        // started sending a packet

                        // led
                        leds_sync_on();
                    break;
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_START_FRAME;
            }

            //==== APP_FLAG_END_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_END_FRAME) {
                // end of frame

                switch (app_vars.state) {

                    case APP_STATE_RX:

                        // done receiving a packet
                        app_vars.packet_len = sizeof(app_vars.packet);

                        // get packet from radio
                        radio_getReceivedFrame(
                            app_vars.packet,
                            &app_vars.packet_len,
                            sizeof(app_vars.packet),
                            &app_vars.rxpk_rssi,
                            &app_vars.rxpk_lqi,
                            &app_vars.rxpk_crc
                        );

                        i = 0;
						
						if ((app_vars.rxpk_crc)) {
							// check if received packet was a join request:
							if ((app_vars.packet[4]==0x00)&&(app_vars.packet[5]==0x44)) { // join req. indicator
							
								// find sender's address:
								app_vars.target_address_msb = app_vars.packet[0];
								app_vars.target_address_lsb = app_vars.packet[1];
								
								radio_rfOff();
								for (j=0;j<0x2FF0;j++);
								
								
								// prepare join req. response:
								app_vars.packet[0] = 0x12;
								app_vars.packet[1] = 0x34;
								app_vars.packet[2] = 0xCA;
								app_vars.packet[3] = 0xFE;
								app_vars.packet[4] = 0x00;
								app_vars.packet[5] = 0x44;
								app_vars.packet[6] = 0X4D;
								app_vars.packet[7] = 0xF9;
								app_vars.packet[8] = 0x15;
								app_vars.packet[9] = 0x66;
								app_vars.packet[10] = 0x66;
								
								// send the packet
								radio_loadPacket(app_vars.packet, 12);
								radio_txEnable();
								radio_txNow();
								
								// return to RX upon TX end frame
								app_vars.state = APP_STATE_TX;
								
								stringToSend[0] = 'A';
								stringToSend[1] = 'C';
								stringToSend[2] = 'K';
								stringToSend[3] = 'A';
								stringToSend[4] = 'C';
								stringToSend[5] = 'K';
								stringToSend[sizeof(stringToSend)-2] = '\r';
								stringToSend[sizeof(stringToSend)-1] = '\n';
								
								// send string over UART
								if (app_vars.uartDone == 1) {
									app_vars.uartDone              = 0;
									app_vars.uart_lastTxByteIndex  = 0;
									uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
								}
								
							}
							else {
							
								memcpy(&stringToSend[i],&app_vars.packet[0],14);
								i += 14;

								stringToSend[sizeof(stringToSend)-2] = '\r';
								stringToSend[sizeof(stringToSend)-1] = '\n';

								// send string over UART
								if (app_vars.uartDone == 1) {
									app_vars.uartDone              = 0;
									app_vars.uart_lastTxByteIndex  = 0;
									uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
								}
							}
						}
						else {
							radio_rxEnable();
							radio_rxNow();
						}

                        // led
                        leds_error_off();
                        break;
						
                    case APP_STATE_TX:
                        // done sending a packet

                        // switch to RX mode
                        radio_rxEnable();
                        radio_rxNow();
                        app_vars.state = APP_STATE_RX;
						
						stringToSend[0] = 'P';
						stringToSend[1] = 'K';
						stringToSend[2] = 'T';
						stringToSend[3] = 'S';
						stringToSend[4] = 'N';
						stringToSend[5] = 'T';
						stringToSend[sizeof(stringToSend)-2] = '\r';
						stringToSend[sizeof(stringToSend)-1] = '\n';
						
						// send string over UART
						if (app_vars.uartDone == 1) {
							app_vars.uartDone              = 0;
							app_vars.uart_lastTxByteIndex  = 0;
							uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
						}

                        // led
                        leds_sync_off();
                        break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            //==== APP_FLAG_TIMER
            if (app_vars.flags & APP_FLAG_TIMER) {
                // timer fired

                if (app_vars.state==APP_STATE_RX) {
                    // stop listening
                    radio_rfOff();

                    // prepare packet
                    app_vars.packet_len = sizeof(app_vars.packet);
                    i = 0;
                    app_vars.packet[0] = 0x12;
                    app_vars.packet[1] = 0x34;
                    app_vars.packet[2] = 0xFF;
                    app_vars.packet[3] = 0xFF;
					app_vars.packet[4] = (uint8_t)(TIMER_PERIOD>>8);
					app_vars.packet[5] = (uint8_t)(TIMER_PERIOD&0xFF);
					app_vars.packet[6] = 0x33;
					app_vars.packet[7] = 0x33;
                    app_vars.packet[8] = CHANNEL;
					
                    // start transmitting packet
                    radio_loadPacket(app_vars.packet,LEN_PKT_TO_SEND);
                    radio_txEnable();
                    radio_txNow();
					
					// restart EB timer
					sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);

                    app_vars.state = APP_STATE_TX;
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_TIMER;
            }
        }
    }
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_START_FRAME;

    // update debug stats
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_END_FRAME;

    // update debug stats
    app_dbg.num_endFrame++;
}

void cb_timer(void) {
    // set flag
    app_vars.flags |= APP_FLAG_TIMER;

    // update debug stats
    app_dbg.num_timer++;

    
}

void cb_uart_tx_done(void) {
    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex<sizeof(stringToSend)) {
        uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
    } else {
        app_vars.uartDone = 1;
    }
}

uint8_t cb_uart_rx(void) {
    uint8_t byte;

    // toggle LED
    leds_error_toggle();

    // read received byte
    byte = uart_readByte();

    // echo that byte over serial
    uart_writeByte(byte);

    return 0;
}
