#include "uart_tx.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "uart.h"

// UART TX byte index.
static uint8_t g_uart_tx_index = 0;

// Pointer to the UART TX buffer.
static const uint8_t* g_uart_tx_buffer = NULL;

// UART TX length.
static size_t g_uart_tx_length = 0;

// UART TX done.
static bool g_uart_tx_done = 0;

// UART TX done callback function.
static void uart_tx_done_callback(void) {
    uart_clearTxInterrupts();
    ++g_uart_tx_index;
    if (g_uart_tx_index < g_uart_tx_length) {
        uart_writeByte(g_uart_tx_buffer[g_uart_tx_index]);
    } else {
        uart_disableInterrupts();
        g_uart_tx_done = true;
    }
}

// UART RX callback function.
static uint8_t uart_rx_callback(void) {
    uart_clearRxInterrupts();
    return 0;
}

void uart_tx_init(void) {
    // Set the UART callback functions.
    uart_setCallbacks(uart_tx_done_callback, uart_rx_callback);
}

bool uart_tx_send(const uint8_t* buffer, const size_t length) {
    if (!uart_tx_send_async(buffer, length)) {
        return false;
    }
    uart_tx_wait();
    return uart_tx_done();
}

bool uart_tx_send_str(const char* buffer) {
    return uart_tx_send((const uint8_t*)buffer, strlen(buffer));
}

bool uart_tx_send_async(const uint8_t* buffer, const size_t length) {
    if (length > UART_TX_MAX_LENGTH) {
        return false;
    }
    g_uart_tx_index = 0;
    g_uart_tx_buffer = buffer;
    g_uart_tx_length = length;
    g_uart_tx_done = false;

    uart_clearTxInterrupts();
    uart_clearRxInterrupts();
    uart_enableInterrupts();
    uart_writeByte(g_uart_tx_buffer[g_uart_tx_index]);
    return true;
}

bool uart_tx_send_str_async(const char* buffer) {
    return uart_tx_send_async((const uint8_t*)buffer, strlen(buffer));
}

void uart_tx_wait(void) {
    while (!g_uart_tx_done)
        ;
}

bool uart_tx_done(void) { return g_uart_tx_done; }
