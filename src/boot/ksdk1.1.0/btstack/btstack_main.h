
#ifndef BLE_ENABLED
#define BLE_ENABLED
#endif

#include "btstack_config.h"
#include "btstack_chipset_cc256x.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_embedded.h"
#include "bluetooth_company_id.h"
#include "classic/btstack_link_key_db.h"
// #include "hal_board.h"
// #include "hal_compat.h"
#include "hal_cpu.h"
#include "hal_tick.h"
// #include "hal_usb.h"
#include "hci.h"
#include "hci_dump.h"

// #include "btstack_defines.h"
// #include "hci_dump.h"
// #include "hci_transport.h"
// #include "devCC2564C.h"
#include "hal_led.h"
#include "led_counter.h"

// const uint8_t cc256x_init_script = "cc2564cInit.c";
// const uint8_t cc256x_init_script = "cc2564cInit.c";

//bt things
void 	ble_writeMenu(void);
void 	ble_setup(void);
void 	ble_enable(void);
void 	ble_disable(void);

