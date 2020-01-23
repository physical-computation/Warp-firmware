 

// TODO: Impliment these functions
 
#include "hal_cpu.h"
// #include "SEGGER_RTT.h"
#include "btstack_config.h"


void hal_cpu_disable_irqs(void){
	SEGGER_RTT_WriteString(0, "\r BLE: CPU Disable IRQs called\n");
}

void hal_cpu_enable_irqs(void){
	SEGGER_RTT_WriteString(0, "\r BLE: CPU Enable IRQs called\n");
}

void hal_cpu_enable_irqs_and_sleep(void){
	SEGGER_RTT_WriteString(0, "\r BLE: CPU Enable IRQs and Sleep called\n");

}