#include "app_timer.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hids.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Define GPIO pins for columns and rows. TODO TODO TODO adafruit board
// static uint8_t row_pins[] = {5, 30, 28, 2, 3, 4};
// static uint8_t col_pins[] = {11, 12, 7, 26, 27, 6, 8};
// #define MATRIX_ANODE_COL

// Define GPIO pins for columns and rows. TODO TODO TODO nordic dongle
// static uint8_t row_pins[] = {20, 22, 24, 32, 9, 10};
// static uint8_t col_pins[] = {2, 47, 45, 42, 13, 15, 17};
// #define MATRIX_ANODE_ROW

// Define GPIO pins for columns and rows. TODO TODO TODO fanstel nrf52810
static uint8_t row_pins[] = {10, 11, 12, 13, 14, 15};
static uint8_t col_pins[] = {24, 25, 26, 27, 28, 29, 30};

// Define macros for size of matrix
#define COL_COUNT sizeof(col_pins)
#define ROW_COUNT sizeof(row_pins)

// Define macros to deal for matrix scanning with respect to the diode
// direction.
#ifdef MATRIX_ANODE_COL
#define MATRIX_CONFIG_ROW() NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false)
#define MATRIX_CONFIG_COL NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO
#define MATRIX_COL_PULL NRF_GPIO_PIN_PULLUP
#define MATRIX_ROW_SET nrfx_gpiote_out_clear
#define MATRIX_COL_IS_SET !nrfx_gpiote_in_is_set
#define MATRIX_ROW_CLEAR nrfx_gpiote_out_set
#else
#define MATRIX_CONFIG_ROW() NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true)
#define MATRIX_CONFIG_COL NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI
#define MATRIX_COL_PULL NRF_GPIO_PIN_PULLDOWN
#define MATRIX_ROW_SET nrfx_gpiote_out_set
#define MATRIX_COL_IS_SET nrfx_gpiote_in_is_set
#define MATRIX_ROW_CLEAR nrfx_gpiote_out_clear
#endif

// Define variables for keyboard bitmap. The bitmap count determines how many
// past bitmaps are stored and is used for debouncing. The bitmap length must be
// large enough for the keyboard matrix.
#define BITMAP_COUNT 5
#define BITMAP_LEN 6
static uint8_t bitmap[BITMAP_COUNT][BITMAP_LEN] = {};
static int bitmap_index = 0;

// Define tag for the SoftDevice BLE configuration.
#define APP_BLE_CONN_CFG_TAG 1

// Define length of the HID input report. It should be the bitmap length.
#define INPUT_REPORT_LEN BITMAP_LEN

// Define timer for matrix scanning and reading battery voltage.
APP_TIMER_DEF(matrix_timer);
APP_TIMER_DEF(battery_timer);

// Define variable for BLE HID service. Allocate memory to support one
// concurrent connection.
BLE_HIDS_DEF(ble_hids, 1, INPUT_REPORT_LEN);

// Define variable for BLE advertising instance.
BLE_ADVERTISING_DEF(ble_adv);

// Define variable for the handle of the BLE connection. Initialize it to an
// invalid handle.
static uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;

// Define variables for keyboard state.
static bool scanning = false;
static bool advertising = false;

// Define variable for battery level.
static nrf_saadc_value_t battery_level = 0;
static nrf_saadc_value_t solar_level = 0;

// Define variable for SAADC state, which is used to coordinate the process for
// sampling separate channels.
static uint8_t saadc_state = 0;

static void whitelist_refresh() {
	// List peers in Peer Manager.
	pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
	uint32_t peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
	pm_peer_id_list_skip_t skip = PM_PEER_ID_LIST_SKIP_NO_ID_ADDR;
	APP_ERROR_CHECK(pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip));
	NRF_LOG_INFO("Whitelist has %d peers out of a maximum %d", peer_id_count, BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

	// Update whitelist.
	APP_ERROR_CHECK(pm_whitelist_set(peer_ids, peer_id_count));
}

static void adv_start(void) {
	APP_ERROR_CHECK(ble_advertising_start(&ble_adv, BLE_ADV_MODE_FAST));
}

static void gap_init(void) {
	// Construct device name using MAC address.
	ble_gap_addr_t ble_addr;
	sd_ble_gap_addr_get(&ble_addr);
	char name[21];
	char *fmt = "ErgoBlue %02x%02x%02x%02x%02x%02x";
	uint8_t *addr = ble_addr.addr;
	sprintf(name, fmt, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

	// Set device name.
	ble_gap_conn_sec_mode_t sec_mode;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	APP_ERROR_CHECK(sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) name, sizeof(name)));

	// Designate device as keyboard.
	APP_ERROR_CHECK(sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD));

	// Use 10ms as the connection interval. Set slave latency to 199 and
	// supervisory timeout to 8 seconds. While we can further increase the slave
	// latency and the supervisory timeout, the improvement in power consumption
	// is negligible.
	ble_gap_conn_params_t gap_conn_params = {
		.min_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
		.max_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
		.slave_latency = 199,
		.conn_sup_timeout = MSEC_TO_UNITS(8000, UNIT_10_MS),
	};
	APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
}

static void hids_init(void) {
	uint8_t report_map[] = {
		0x05, 0x01, // Usage Page (Generic Desktop)
		0x09, 0x06, // Usage (Keyboard)
		0xa1, 0x01, // Collection (Application)
		0x95, 0x08, // Report Count (8)
		0x75, 0x08, // Report Size (8)
		0x81, 0x01, // Input (Constant)
		0xc0, // End Collection (Application)
	};

	// Define input report. Since there is only one, there is no need to define
	// an array and we can just use the pointer of the variable.
	ble_hids_inp_rep_init_t p_input_report = {
		.max_len = INPUT_REPORT_LEN,
		.rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT,
		.sec.cccd_wr = SEC_JUST_WORKS,
		.sec.wr = SEC_JUST_WORKS,
		.sec.rd = SEC_JUST_WORKS,
	};

	// Initialize HID Service
	ble_hids_init_t hids_init_obj = {
		.is_kb = true,

		// Include input report.
		.p_inp_rep_array = &p_input_report,
		.inp_rep_count = 1,

		// Include report map.
		.rep_map.p_data = report_map,
		.rep_map.data_len = sizeof(report_map),

		// Define HID version.
		.hid_information.bcd_hid = 0x0101,

		// Define HID flags for automatic reconnection.
		.hid_information.flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK,

		// Define HID security.
		.rep_map.rd_sec = SEC_JUST_WORKS,
		.hid_information.rd_sec = SEC_JUST_WORKS,
		.boot_kb_inp_rep_sec.cccd_wr = SEC_JUST_WORKS,
		.boot_kb_inp_rep_sec.rd = SEC_JUST_WORKS,
		.boot_kb_outp_rep_sec.rd = SEC_JUST_WORKS,
		.boot_kb_outp_rep_sec.wr = SEC_JUST_WORKS,
		.protocol_mode_rd_sec = SEC_JUST_WORKS,
		.protocol_mode_wr_sec = SEC_JUST_WORKS,
		.ctrl_point_wr_sec = SEC_JUST_WORKS,
	};
	ble_hids_init(&ble_hids, &hids_init_obj);
}

static void adv_evt_handler(ble_adv_evt_t ble_adv_evt) {
	switch (ble_adv_evt) {

	// Maintain advertising state in variable. The event will be
	// BLE_ADV_EVT_WHITELIST with an empty whitelist and
	// BLE_ADV_EVT_FAST_WHITELIST otherwise.
	case BLE_ADV_EVT_FAST:
	case BLE_ADV_EVT_FAST_WHITELIST:
		NRF_LOG_INFO("Advertising has started");
		advertising = true;
		break;

	case BLE_ADV_EVT_IDLE:
		NRF_LOG_INFO("Advertising has stopped");
		advertising = false;
		break;

	// Respond with whitelist when requested by advertising module. If the
	// whitelist is empty, the advertising module will continue without one.
	case BLE_ADV_EVT_WHITELIST_REQUEST: {
		// Get whitelisted peers from Peer Manager.
		ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
		ble_gap_irk_t whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
		uint32_t addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
		uint32_t irk_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
		APP_ERROR_CHECK(pm_whitelist_get(whitelist_addrs, &addr_cnt, whitelist_irks, &irk_cnt));
		NRF_LOG_INFO("Whitelist response with %d addresses and %d IRKs", addr_cnt, irk_cnt);

		// Respond to advertising module with whitelist.
		APP_ERROR_CHECK(ble_advertising_whitelist_reply(&ble_adv, whitelist_addrs, addr_cnt, whitelist_irks, irk_cnt));
		break;
	}

	default:
		break;

	}
}

static void adv_init(void) {
	ble_uuid_t adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};
	ble_advertising_init_t init = {
		.advdata.name_type = BLE_ADVDATA_FULL_NAME,
		.advdata.include_appearance = true,
		.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
		.advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]),
		.advdata.uuids_complete.p_uuids = adv_uuids,
		.evt_handler = adv_evt_handler,

		// Use whitelist to only allow connection from a single central. TODO
		// TODO TODO directed not useful against a malicious central. also think
		// about process of adding additional centrals.
		.config.ble_adv_whitelist_enabled = true,

		// Define advertising interval and duration. TODO TODO TODO duration
		// will change depending on context.
		.config.ble_adv_fast_enabled = true,
		.config.ble_adv_fast_interval = MSEC_TO_UNITS(25, UNIT_0_625_MS),
		.config.ble_adv_fast_timeout = MSEC_TO_UNITS(3000, UNIT_10_MS),
	};
	APP_ERROR_CHECK(ble_advertising_init(&ble_adv, &init));
	ble_advertising_conn_cfg_tag_set(&ble_adv, APP_BLE_CONN_CFG_TAG);

	// Increase transmit power to 4dBm. TODO TODO TODO dynamically determine
	// maximum? always use maximum? iirc minimum power difference.
	APP_ERROR_CHECK(sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, ble_adv.adv_handle, 4));
}

static void conn_params_init(void) {
	// Configure connection parameter negotiation using default values from the
	// keyboard example.
	ble_conn_params_init_t cp_params = {
		.first_conn_params_update_delay = APP_TIMER_TICKS(5000),
		.next_conn_params_update_delay = APP_TIMER_TICKS(30000),
		.max_conn_params_update_count = 3,
		.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID,
	};
	APP_ERROR_CHECK(ble_conn_params_init(&cp_params));
}

static void pm_evt_handler(pm_evt_t const *p_evt) {
	// Call peer manager event handlers.
	pm_handler_on_pm_evt(p_evt);
	pm_handler_flash_clean(p_evt);
	pm_handler_disconnect_on_sec_failure(p_evt);

	switch (p_evt->evt_id) {

	// Start advertising if peers were just deleted. TODO TODO TODO iirc last
	// time this didn't work?
	case PM_EVT_PEERS_DELETE_SUCCEEDED:
		adv_start();
		break;

	// Upon establishing a new bond, add peer to whitelist.
	case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
		if (p_evt->params.peer_data_update_succeeded.flash_changed) {
			if (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING) {
				NRF_LOG_INFO("Adding peer to whitelist");
				whitelist_refresh();
			}
		}
		break;

	default:
		break;

	}
}

static void peer_manager_init(void) {
	APP_ERROR_CHECK(pm_init());

	// TODO TODO TODO mitm with passkey or something if we can get it working on
	// Linux/BlueZ or OOB if sending to another Nordic device.
	ble_gap_sec_params_t sec_param = {
		.bond = 1,
		.io_caps = BLE_GAP_IO_CAPS_NONE,
		.min_key_size = 16,
		.max_key_size = 16,
		.kdist_own.enc = 1,
		.kdist_own.id = 1,
		.kdist_peer.enc = 1,
		.kdist_peer.id = 1,

		// Enable LE Secure Connections. See https://bit.ly/2Ft9MWT and
		// https://bit.ly/2ZIIL9y for information regarding security of
		// connection established with LESC. LESC provides protection against
		// passive eavesdrop but not against active attacks.
		.lesc = 1,
	};

	APP_ERROR_CHECK(pm_sec_params_set(&sec_param));
	APP_ERROR_CHECK(pm_register(pm_evt_handler));
}

static void send_hid_bitmap(uint8_t *data) {
	// TODO TODO TODO queue? explain no error check here.
	ble_hids_inp_rep_send(&ble_hids, 0, INPUT_REPORT_LEN, data, conn_handle);
}

static void scan_hid_handler(void *p_context) {
	// Scan matrix and set the appropriate bits.
	for (int i = 0; i < ROW_COUNT; i++) {
		MATRIX_ROW_SET(row_pins[i]);
		for (int j = 0; j < COL_COUNT; j++) {
			if (MATRIX_COL_IS_SET(col_pins[j])) {
				uint8_t bit = i*COL_COUNT + j;
				bitmap[bitmap_index][bit / 8] |= 1 << (bit % 8);
			}
		}
		MATRIX_ROW_CLEAR(row_pins[i]);
	}

	// Set bitmap0 to be OR of everything except the last bitmap and set bitmap1
	// to be OR of everything except the first bitmap. We can consider then the
	// change from bitmap0 to bitmap1. This lets us detect key down events as
	// soon as they occur and deal with debouncing later. This does introduce a
	// small latency for key up events.
	uint8_t bitmap0[BITMAP_LEN], bitmap1[BITMAP_LEN];
	memcpy(bitmap0, bitmap[(bitmap_index+1)%BITMAP_COUNT], BITMAP_LEN);
	memcpy(bitmap1, bitmap[bitmap_index], BITMAP_LEN);
	for (int i = 2; i < BITMAP_COUNT; i++) {
		uint8_t *bitmap2 = bitmap[(bitmap_index+i)%BITMAP_COUNT];
		for (int j = 0; j < BITMAP_LEN; j++) {
			bitmap0[j] |= bitmap2[j];
			bitmap1[j] |= bitmap2[j];
		}
	}

	// If the bitmap changed and a client is connected, send new bitmap. If no
	// client is connected and not currently advertising, begin advertising.
	if (memcmp(bitmap0, bitmap1, BITMAP_LEN)) {
		NRF_LOG_INFO("Matrix has HID data");
		if (conn_handle != BLE_CONN_HANDLE_INVALID) {
			send_hid_bitmap(bitmap1);
		} else {
			if (!advertising) {
				adv_start();
			}
		}
	}

	// Increment position in bitmap array.
	bitmap_index++;
	bitmap_index %= BITMAP_COUNT;
	memset(bitmap[bitmap_index], 0, BITMAP_LEN);

	// If some key remains active, schedule next scan for 8ms later.
	uint8_t zeros[BITMAP_LEN] = {};
	if (memcmp(bitmap1, zeros, BITMAP_LEN)) {
		APP_ERROR_CHECK(app_timer_start(matrix_timer, APP_TIMER_TICKS(8), NULL));
		return;
	}

	// If no key is active, set all pins to high and stop matrix scanning.
	for (int i = 0; i < ROW_COUNT; i++) {
		MATRIX_ROW_SET(row_pins[i]);
	}
	scanning = false;
	NRF_LOG_INFO("Matrix scanning has stopped");
}

static void saadc_buffer_convert(nrf_saadc_input_t pin, nrf_saadc_value_t *value) {
	nrf_saadc_channel_config_t channel = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(pin);

	// Set acquisition time. A larger value gives a more stable reading. It also
	// uses more power, though that's a minor concern as we rarely use SAADC. A
	// larger value also allows for additional resistance between the input pin
	// and the power source.
	channel.acq_time = NRF_SAADC_ACQTIME_40US;

	// Enable burst mode, which is necessary for oversampling.
	channel.burst = NRF_SAADC_BURST_ENABLED;

	// TODO TODO TODO explain. probably want this (and the adc) as constants.
	channel.gain = NRF_SAADC_GAIN1_2;

	// Initialize channel and read SAADC value asynchronously.
	APP_ERROR_CHECK(nrfx_saadc_channel_init(0, &channel));
	APP_ERROR_CHECK(nrfx_saadc_buffer_convert(value, 1));
	APP_ERROR_CHECK(nrfx_saadc_sample());
}

static void saadc_evt_handler(nrfx_saadc_evt_t const *p_event) {
	// When calibration finishes, read VDD into the battery voltage.
	if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
		saadc_buffer_convert(NRF_SAADC_INPUT_VDD, &battery_level);
		return;

	}

	// After reading the battery voltage, read the solar voltage. If both have
	// been read, send voltage data to client if one is connected. Set the most
	// significant bit to designate that the message contains voltage data and
	// not key data.
	if (p_event->type == NRFX_SAADC_EVT_DONE) {
		saadc_state++;
		if (saadc_state == 1) {
			saadc_buffer_convert(NRF_SAADC_INPUT_AIN0, &solar_level);
		} else if (saadc_state == 2) {
			uint64_t i = ((uint64_t) 1 << 47) | (battery_level << 16) | solar_level;
			uint8_t data[INPUT_REPORT_LEN];
			memcpy(data, &i, INPUT_REPORT_LEN);
			send_hid_bitmap(data);
			saadc_state = 0;
		}
	}
}

static void battery_handler(void *p_context) {
	// Calibrate to initialize reading. TODO TODO TODO only if connected
	if (conn_handle != BLE_CONN_HANDLE_INVALID) {
		APP_ERROR_CHECK(nrfx_saadc_calibrate_offset());
	}
}

static void timer_init(void) {
	APP_ERROR_CHECK(app_timer_init());

	// Initialize timer for matrix scanning.
	APP_ERROR_CHECK(app_timer_create(&matrix_timer, APP_TIMER_MODE_SINGLE_SHOT, scan_hid_handler));

	// Read voltages once every 5 minutes.
	APP_ERROR_CHECK(app_timer_create(&battery_timer, APP_TIMER_MODE_REPEATED, battery_handler));
	APP_ERROR_CHECK(app_timer_start(battery_timer, APP_TIMER_TICKS(5*60*1000), NULL));
}

static void gpio_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	// If matrix scan is not in progress, unset row pins and start matrix scan.
	if (!scanning) {
		NRF_LOG_INFO("Matrix scanning has started");
		scanning = true;
		for (int i = 0; i < ROW_COUNT; i++) {
			MATRIX_ROW_CLEAR(row_pins[i]);
		}
		scan_hid_handler(NULL);
	}
}

static void gpio_init(void) {
	// Initialize GPIO library.
	APP_ERROR_CHECK(nrfx_gpiote_init());

	// Initialize rows as output with initial voltage.
	nrfx_gpiote_out_config_t out_config = MATRIX_CONFIG_ROW();
	for (int i = 0; i < ROW_COUNT; i++) {
		APP_ERROR_CHECK(nrfx_gpiote_out_init(row_pins[i], &out_config));
	}

	// Initialize rows as input with pull resistor.
	nrfx_gpiote_in_config_t in_config = MATRIX_CONFIG_COL(false);
	in_config.pull = MATRIX_COL_PULL;
	for (int i = 0; i < COL_COUNT; i++) {
		APP_ERROR_CHECK(nrfx_gpiote_in_init(col_pins[i], &in_config, gpio_evt_handler));
		nrfx_gpiote_in_event_enable(col_pins[i], true);
	}
}

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
	// Call peer manager event handlers.
	pm_handler_secure_on_connection(p_ble_evt);
	pm_handler_secure_on_error(p_ble_evt);

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		advertising = false;
		NRF_LOG_INFO("Client has connected");
		break;
	case BLE_GAP_EVT_DISCONNECTED:
		conn_handle = BLE_CONN_HANDLE_INVALID;
		NRF_LOG_INFO("Client has disconnected");
		break;
	}
}

static void ble_init(void) {
	// Enable SoftDevice.
	APP_ERROR_CHECK(nrf_sdh_enable_request());

	// Get default BLE configuration and enable BLE stack.
	uint32_t ram_start = 0;
	APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start));
	APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(ble_observer, 3, ble_evt_handler, NULL);

	// Define GATT instance and initialize GATT.
	NRF_BLE_GATT_DEF(ble_gatt);
	APP_ERROR_CHECK(nrf_ble_gatt_init(&ble_gatt, NULL));

	// Initialize everything else.
	gap_init();
	adv_init();
	conn_params_init();
	peer_manager_init();
}

static void saadc_init(void) {
	// Configure and initialize SAADC. Use maximum resolution and oversampling.
	nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
	saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;
	saadc_config.oversample = NRF_SAADC_OVERSAMPLE_256X;
	saadc_config.low_power_mode = true;
	APP_ERROR_CHECK(nrfx_saadc_init(&saadc_config, saadc_evt_handler));
}

int main(void) {
	// Initialize logging.
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	// Initialize everything else.
	saadc_init();
	timer_init();
	gpio_init();
	ble_init();
	hids_init();

	// Enable DC/DC regular to reduce energy consumption.
	APP_ERROR_CHECK(sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE));
	// APP_ERROR_CHECK(sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE));

	// Delete peers if necessary. In such case, advertising will begin after
	// that finishes. Otherwise, load whitelist and begin advertising. TODO TODO
	// TODO
	// bool delete_peers = true;
	bool delete_peers = false;
	if (delete_peers) {
		APP_ERROR_CHECK(pm_peers_delete());
	} else {
		whitelist_refresh();
		adv_start();
	}

	// Enter main loop.
	NRF_LOG_INFO("ErgoBlue firmware is running");
	while (true) {
		APP_ERROR_CHECK(nrf_ble_lesc_request_handler());
		if (NRF_LOG_PROCESS() == false) {
			nrf_pwr_mgmt_run();
		}
	}
}
