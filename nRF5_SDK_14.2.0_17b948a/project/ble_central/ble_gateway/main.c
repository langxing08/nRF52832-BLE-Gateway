/**
 * BLE_Gateway working as Central, scans Peripheral and then analysis ADV_DATA and SCAN_RESP_DATA.
 * When get Device Name, RSSI and MAC Address, BLE_Gateway will send this information via UART.
 * Uart baud 115200bps, 8 data bit, 1 stop bit, none check bit, none flow control.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "bsp_config.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_wdt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define HARDWARE_NUMBER			"HW_4.3"
#define SOFTWARE_NUMBER			"SW_1.0.4"
#define FIRMWARE_NUMBER			"FW_14.2.0"

#define APP_BLE_CONN_CFG_TAG    1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define SCAN_INTERVAL           MSEC_TO_UNITS(100, UNIT_0_625_MS)		/**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             MSEC_TO_UNITS(50, UNIT_0_625_MS)		/**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */

#define WDT_FEED_INTERVALY		APP_TIMER_TICKS(800)					/**< WDT feed interval. */

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE NUS service client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */

APP_TIMER_DEF(wdt_feed_timer_id);									/**< WDT feed delay timer instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

nrf_drv_wdt_channel_id m_channel_id;	// wdt channel id

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};


#define USER_FILTER_MAX_INDEX	20
#define USER_FILTER_LEN			3

//user filter
typedef struct
{
	uint8_t cmd_enable;
	uint8_t filter_keywords[USER_FILTER_LEN];
	uint8_t filter_length;
} user_filter_t;

user_filter_t user[USER_FILTER_MAX_INDEX];

static uint8_t filter_enable = 0;	// user device name filter enable


/**@brief Function for convert Bluetooth MAC address to string.
 *
 * @param[in] *pAddr - Bluetooth MAC address.
 * 
 * @param[out] BD address as a string.
 */
static char *Util_convertBdAddr2Str(uint8_t const *pAddr)
{
	uint8_t     charCnt;
	char        hex[] = "0123456789ABCDEF";
	static char str[(BLE_GAP_ADDR_LEN << 1)+1];
	char        *pStr = str;

	//Start from end of addr
	pAddr += BLE_GAP_ADDR_LEN;

	for (charCnt = BLE_GAP_ADDR_LEN; charCnt > 0; charCnt--)
	{
		*pStr++ = hex[*--pAddr >> 4];
		*pStr++ = hex[*pAddr & 0x0F];
	}

	pStr = NULL;

	return str;
}


/**@brief Function for process AT command.
 *
 * @param[in] *pBuffer  pointer of string.
 * @param[in] length	Length of the data.
 */
static void AT_cmd_handle(uint8_t *pBuffer, uint16_t length)
{
	ret_code_t err_code;
	
	// check whether is AT cmd or not	
	if(length < 2) 
		return;
	if(strncmp((char*)pBuffer, "AT", 2) != 0)
		return;
	
	// AT test: AT?\r\n
	if((length == 5) && (strncmp((char*)pBuffer, "AT?\r\n", 5) == 0))
	{
		printf("AT:OK\r\n");
	}
	
	// System soft reset: AT+RESET\r\n
	else if((length == 10) && (strncmp((char*)pBuffer, "AT+RESET\r\n", 10) == 0))
	{
		NVIC_SystemReset();	// Restart the system by default	
	}
	
	// Hardware/firmware/software version check: AT+VER?\r\n
	else if((length == 9) && (strncmp((char*)pBuffer, "AT+VER?\r\n", 9) == 0))
	{
		printf("AT+VER:%s,%s,%s\r\n", HARDWARE_NUMBER, FIRMWARE_NUMBER, SOFTWARE_NUMBER);	
	}	

	// MAC address check: AT+MAC?\r\n
	else if((length == 9) && (strncmp((char*)pBuffer, "AT+MAC?\r\n", 9) == 0))//check MAC addr
	{
		ble_gap_addr_t device_addr;
	
		// Get BLE address.
		#if (NRF_SD_BLE_API_VERSION >= 3)
			err_code = sd_ble_gap_addr_get(&device_addr);
		#else
			err_code = sd_ble_gap_address_get(&device_addr);
		#endif
		APP_ERROR_CHECK(err_code);

		printf("AT+MAC:%s\r\n", Util_convertBdAddr2Str(device_addr.addr));
	}	
	
	// Filter enable: AT+FILTER=N\r\n, 1:enable, 0:disable
	else if((length == 13) && (strncmp((char*)pBuffer, "AT+FILTER=", 10) == 0))
	{
		uint32_t filter_enable_tmp;
		sscanf((char*)pBuffer, "AT+FILTER=%x\r\n", &filter_enable_tmp);
		if((filter_enable_tmp == 0) || (filter_enable_tmp == 1))
		{
			filter_enable = filter_enable_tmp;
			printf("AT+FILTER:OK\r\n");
		}
		else
		{
			printf("AT+FILTER:ERP\r\n");
		}
	}
	
	// User filter format rule:AT+USER=M,N,XYZ\r\n
	else if((length >= 15) && (strncmp((char*)pBuffer, "AT+USER=", 8) == 0))
	{
		uint32_t user_cmd_index_tmp, user_cmd_enable_tmp;
		
		sscanf((char*)pBuffer, "AT+USER=%x,%x,", 
				&user_cmd_index_tmp, 	// user cmd index range
				&user_cmd_enable_tmp);	// user cmd enable:0:disable, 1:enable
		if((user_cmd_index_tmp < USER_FILTER_MAX_INDEX)
			&&((user_cmd_enable_tmp == 0) ||(user_cmd_enable_tmp == 1)))
		{
			user[user_cmd_index_tmp].cmd_enable = user_cmd_enable_tmp;
			
			char* ans;
			ans = strrchr((char*)pBuffer, ',');
			ans++;	// atfter ','
			
			memcpy(user[user_cmd_index_tmp].filter_keywords, ans, USER_FILTER_LEN);
			
			char *p = strstr(ans, "\r\n");
			user[user_cmd_index_tmp].filter_length = p - ans; 
						
			printf("AT+USER:OK\r\n");
		}
		else
		{
			printf("AT+USER:ERP\r\n");
		}
	}	
}


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

			#if 1	// uart printf back uart received data
				printf("HHH:%s\r\n", data_array);
			#endif
				
				AT_cmd_handle(data_array, index);

                index = 0;
            }
            break;

        default:
            break;
    }
}


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
			
			uint8_t buff[60];
			uint8_t device_name[BLE_GAP_ADV_MAX_SIZE + 1];
			
			uint8_t filter_flag = 0;	// find device name when filter enable
			uint8_t no_filter_flag = 0;	// find device name when filter disable
			
			uint8_t ad_len;		// AD Length in a AD Structure
			uint8_t ad_type;	// AD Type   in a AD Structure
			uint8_t index = 0;
			
			// search for device name
			// AD Structure include AD Length | AD Type | AD Data
			while((index < (BLE_GAP_ADV_MAX_SIZE - 1)) && (index < p_adv_report->dlen))
			{
				ad_len = p_adv_report->data[index] - 1;
				ad_type = p_adv_report->data[index + 1];
				if((ad_type == 0x08) || (ad_type == 0x09)) 
				{
					memcpy(device_name, &p_adv_report->data[index + 2], ad_len);
					device_name[ad_len] = '\0';
					
					if(filter_enable)
					{
						// device name filter(first USER_FILTER_LEN Bytes)
						for(uint8_t cnt=0; cnt < USER_FILTER_MAX_INDEX; cnt++)
						{
							if(user[cnt].cmd_enable 
								&& !memcmp(device_name, user[cnt].filter_keywords, user[cnt].filter_length))
							{
								filter_flag = 1;
								break;
							}
						}
					}
					else
					{
						no_filter_flag = 1;
					}//if(filter_enable)
					
					if(filter_flag || no_filter_flag)
					{
						memset(buff, 0, sizeof(buff));
						sprintf((char*)buff, "%s,%d,9,%s\r\n", // MAC address, RSSI, 9, device name
								Util_convertBdAddr2Str(p_adv_report->peer_addr.addr),
								p_adv_report->rssi,
								device_name);
						
						// uart send
						for (uint8_t i = 0; i < strlen((char*)buff); i++)
						{
							do
							{
								err_code = app_uart_put(buff[i]);
								if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
								{
									NRF_LOG_INFO("Failed uart tx message. Error 0x%x. ", err_code);
									APP_ERROR_CHECK(err_code);
								}
							} while (err_code == NRF_ERROR_BUSY);
						}
					}//end if(filter_flag || no_filter_flag)
					
					break;
				}
				else
				{
					index += p_adv_report->data[index] + 1;
				}
			}
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
			
        default:
            break;
    }
}


/**@brief Function for feed WDT.
 *
 * @param[in] p_context   Unused.
 */
static void wdt_feed_timer_handler(void * p_context)
{
	nrf_drv_wdt_channel_feed(m_channel_id);
}


/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    // NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}


/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&wdt_feed_timer_id, 
								APP_TIMER_MODE_REPEATED, 
								wdt_feed_timer_handler);
    APP_ERROR_CHECK(err_code);		
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the wdt module. */
void wdt_init(void)
{
	// Configure WDT.
	uint32_t err_code = NRF_SUCCESS;
	
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
	
    nrf_drv_wdt_enable();
}


/**@brief Function for wdt feed timers.
 *
 * @details Starts application wdt feed timers.
 */
static void wdt_feed_timers_start(void)
{
	ret_code_t err_code;

    err_code = app_timer_start(wdt_feed_timer_id, WDT_FEED_INTERVALY, NULL);
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    log_init();
    timer_init();
    power_init();
    uart_init();
	wdt_init();
    buttons_leds_init();	
    ble_stack_init();
    gatt_init();

    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    printf("BLE Gateway started.\r\n");
    NRF_LOG_INFO("BLE Gateway started.");
	
	wdt_feed_timers_start();
	
    scan_start();

    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}
