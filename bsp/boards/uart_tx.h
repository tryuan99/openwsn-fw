// The UART TX module is intended for transmitting debug messages over UART.

#ifndef __UART_TX_H
#define __UART_TX_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// Maximum UART TX length in bytes.
#define UART_TX_MAX_LENGTH 40

// Initialize UART TX.
void uart_tx_init(void);

// Send the buffer over UART synchronously.
bool uart_tx_send(const uint8_t* buffer, size_t length);
bool uart_tx_send_str(const char* buffer);

// Send the buffer over UART asynchronously. The buffer should not be modified
// until the TX is done.
bool uart_tx_send_async(const uint8_t* buffer, size_t length);
bool uart_tx_send_str_async(const char* buffer);

// Wait until the previous UART TX is done.
void uart_tx_wait(void);

// Return whether the previous UART TX is done.
bool uart_tx_done(void);

#endif  // __UART_TX_H
