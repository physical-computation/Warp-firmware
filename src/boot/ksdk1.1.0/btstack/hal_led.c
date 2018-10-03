
#include "SEGGER_RTT.h"

void hal_led_toggle(void){
    SEGGER_RTT_printf(0, "\r\t * LED Toggled * \n");
}
