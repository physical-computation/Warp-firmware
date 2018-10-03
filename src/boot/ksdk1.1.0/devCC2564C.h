
static btstack_packet_callback_registration_t hci_event_callback_registration;
static int getDeviceIndexForAddress(bd_addr_t addr);

static void start_scan(void);
static int has_more_remote_name_requests(void);
static void do_next_remote_name_request(void);
static void continue_remote_names(void);
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

int btstack_main(int argc, const char *argv[]);
