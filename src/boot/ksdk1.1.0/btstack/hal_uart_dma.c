// TODO:

#include "hal_uart_dma.h"
#include "SEGGER_RTT.h"


/**
 * @brief Init and open device
 */
void hal_uart_dma_init(void){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_init called\n");
}

/**
 * @brief Set callback for block received - can be called from ISR context
 * @param callback
 */
void hal_uart_dma_set_block_received( void (*callback)(void)){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_set_block_received called\n");
}

/**
 * @brief Set callback for block sent - can be called from ISR context
 * @param callback
 */
void hal_uart_dma_set_block_sent( void (*callback)(void)){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_set_block_sent called\n");

}

/**
 * @brief Set baud rate
 * @note During baud change, TX line should stay high and no data should be received on RX accidentally
 * @param baudrate
 */
int  hal_uart_dma_set_baud(uint32_t baud){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_set_baud called\n");

}

/**
 * @brief Send block. When done, callback set by hal_uart_set_block_sent must be called
 * @param buffer
 * @param lengh
 */
void hal_uart_dma_send_block(const uint8_t *buffer, uint16_t length){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_send_block called\n");

}

/**
 * @brief Receive block. When done, callback set by hal_uart_dma_set_block_received must be called
 * @param buffer
 * @param lengh
 */
void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_receive_block called\n");

}

/**
 * @brief Set or clear callback for CSR pulse - can be called from ISR context
 * @param csr_irq_handler or NULL to disable IRQ handler
 */
void hal_uart_dma_set_csr_irq_handler( void (*csr_irq_handler)(void)){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_set_csr_irq_handler called\n");

}

/**
 * @brief Set sleep mode
 * @param block_received callback
 */
void hal_uart_dma_set_sleep(uint8_t sleep){
	SEGGER_RTT_WriteString(0, "\r BLE: hal_uart_dma_set_sleep called\n");

}


