/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "ble_image_transfer_service_c.h"

#include "nrf_ble_whitelist.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SCAN_WITH_WHITELIST_ENABLED 1
#define NRF_BLE_SCAN_ACTIVE_SCANNING 0                                  /**< 0 -- passive scanning, 1 -- active scanning. */
//#define SCAN_MATCH_WITH_CONNECT

#define SEND_CMD_BUTTON_PIN  BSP_BUTTON_0
#define SEND_DATA_BUTTON_PIN BSP_BUTTON_1
#define CHANGE_ADV_PAYLOAD_PIN BSP_BUTTON_2

#define SCANNING_LED BSP_BOARD_LED_0                                    /**< Is on when device is scanning. */
#define CONNECTED_LED BSP_BOARD_LED_1                                   /**< Is on when device has connected. */
#define DATA_TX_LED  BSP_BOARD_LED_2                                    /**< Is on when service is sending data. */
#define DATA_RX_LED  BSP_BOARD_LED_3                                    /**< Is on when service is receiving data */

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)                      /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

//#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */
#define ITS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic Image Transfer Service (vendor specific). */

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_ITS_C_DEF(m_ble_its_c);                                             /**< BLE Nordic Image Transfer Service (NUS) client instance. */

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

#define SCAN_DURATION_WHITELIST      0                                  /**< Duration of the scanning in units of 10 milliseconds. */

#define TX_POWER_LEVEL (0)                                              /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE       /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 20                                             /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 10                                             /**< Maximum number of events in the scheduler queue. */
#endif



#define MAX_LEN_DEVICE_ID          NRF_BLE_ADV_MANUFACTURER_SPECIFIC_DEVICE_ID_LENGTH
#define MAX_LEN_DEVICE_NAME        NRF_BLE_ADV_MANUFACTURER_SPECIFIC_DEVICE_NAME_LENGTH

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic Image Transfer Service module. */

/**@brief ITS UUID. */
static ble_uuid_t const m_its_uuid =
{
        .uuid = BLE_UUID_ITS_SERVICE,
        .type = ITS_SERVICE_UUID_TYPE
};

#define MAX_LEN_DEVICE_ID       8
#define MAX_LEN_DEVICE_NAME     12

static uint8_t m_manufacturer_specific_target_device_name[MAX_LEN_DEVICE_NAME] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C };
static char m_target_periph_name[MAX_LEN_DEVICE_NAME] = "DEVICE_000001"; /**< Name of the device we try to connect to. This name is searched in the scan report data*/

#ifdef SCAN_WITH_WHITELIST_ENABLED
static bool m_whitelist_disabled = false;                           /**< True if the whitelist is temporarily disabled. */
#else
static bool m_whitelist_disabled = true;                           /**< True if the whitelist is temporarily disabled. */
#endif

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t m_scan_param =
{
        .active        = NRF_BLE_SCAN_ACTIVE_SCANNING,           /* Disable the acvtive scanning */
        .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
        .window        = NRF_BLE_SCAN_SCAN_WINDOW,
        .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
        .timeout       = SCAN_DURATION_WHITELIST,
        .scan_phys     = BLE_GAP_PHY_1MBPS,
};

static ble_its_c_img_info_t m_transfer_image_info;
static ble_its_c_img_info_t m_received_image_info;

static ble_its_cmd_data_t m_its_tx_cmd_data;
static ble_its_cmd_data_t m_its_rx_cmd_data;


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_connected_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}

static void tx_power_scanning_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
        ret_code_t ret;

        ret = nrf_ble_scan_start(&m_scan);
        APP_ERROR_CHECK(ret);

        ret = bsp_indication_set(BSP_INDICATE_SCANNING);
        APP_ERROR_CHECK(ret);

        NRF_LOG_INFO("Scanning Start");

        tx_power_scanning_set();
}


static void on_whitelist_req(void)
{
        ret_code_t ret;
        if (((nrf_ble_whitelist_cnt() == 0)) ||
            (m_whitelist_disabled))
        {
                m_scan_param.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
                ret = nrf_ble_scan_params_set(&m_scan, &m_scan_param);
                APP_ERROR_CHECK(ret);
        }
        else
        {
                ret = nrf_ble_whitelist_enable();
                APP_ERROR_CHECK(ret);
        }
}

/**@brief Function for disabling the use of the whitelist for scanning.
 */
static void whitelist_disable(void)
{
        if (!m_whitelist_disabled)
        {
                NRF_LOG_INFO("Whitelist temporarily disabled.");
                m_whitelist_disabled = true;
                nrf_ble_scan_stop();
                scan_start();
        }
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
        ret_code_t err_code;

        switch(p_scan_evt->scan_evt_id)
        {
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
                on_whitelist_req();
                m_whitelist_disabled = false;
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
                //NRF_LOG_INFO("NRF_BLE_SCAN_EVT_FILTER_MATCH");
                //p_scan_evt->params.connected.p_connected.

                break;

        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:

                NRF_LOG_INFO("NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT");
                //                NRF_LOG_HEXDUMP_INFO(p_scan_evt->params.p_whitelist_adv_report, sizeof(p_scan_evt->params.p_whitelist_adv_report));

                break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const * p_connected =
                        p_scan_evt->params.connected.p_connected;
                // Scan is automatically stopped by the connection.
                NRF_LOG_INFO("Connecting to target peripheral : 0x%02x%02x%02x%02x%02x%02x",
                             p_connected->peer_addr.addr[5],
                             p_connected->peer_addr.addr[4],
                             p_connected->peer_addr.addr[3],
                             p_connected->peer_addr.addr[2],
                             p_connected->peer_addr.addr[1],
                             p_connected->peer_addr.addr[0]
                             );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
        } break;

        default:
                break;
        }
}

static void set_test_whitelist(void)
{
        ble_gap_addr_t whitelist_addrs;
        uint8_t whitelist_number = 0;
        ret_code_t err_code;

        /* Gateway 1 */
        whitelist_addrs.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
        whitelist_addrs.addr_id_peer = 0x00;
//        whitelist_addrs.addr[5]   = 0xCC;
//        whitelist_addrs.addr[4]   = 0xD1;
//        whitelist_addrs.addr[3]   = 0x6E;
//        whitelist_addrs.addr[2]   = 0x0A;
//        whitelist_addrs.addr[1]   = 0x98;
//        whitelist_addrs.addr[0]   = 0x01;

        whitelist_addrs.addr[5]   = 0xcc;
        whitelist_addrs.addr[4]   = 0xcc;
        whitelist_addrs.addr[3]   = 0xcc;
        whitelist_addrs.addr[2]   = 0xcc;
        whitelist_addrs.addr[1]   = 0xcc;
        whitelist_addrs.addr[0]   = 0xcc;

        err_code = nrf_ble_whitelist_add(&whitelist_addrs, &whitelist_number);
        APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.p_scan_param     = &m_scan_param;

#ifdef SCAN_MATCH_WITH_CONNECT
        init_scan.connect_if_match = true;
#else
        init_scan.connect_if_match = false;
#endif
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

#if (NRF_BLE_SCAN_FILTER_ENABLE == 1)
        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_its_uuid);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_MANUFACTURER_FILTER, m_manufacturer_specific_target_device_name);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_MANUFACTURER_SPECIFIC_FILTER, false);
        APP_ERROR_CHECK(err_code);
#endif

#ifdef SCAN_WITH_WHITELIST_ENABLED
        set_test_whitelist();
        m_whitelist_disabled = false;
#else
        m_whitelist_disabled = true;
#endif

}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                      p_evt->conn_handle,
                      p_evt->conn_handle);
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
        ble_its_c_on_db_disc_evt(&m_ble_its_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t ret_val;

        NRF_LOG_DEBUG("Receiving data.");
        NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

        for (uint32_t i = 0; i < data_len; i++)
        {
                do
                {
                        ret_val = app_uart_put(p_data[i]);
                        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                        {
                                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                                APP_ERROR_CHECK(ret_val);
                        }
                } while (ret_val == NRF_ERROR_BUSY);
        }
        if (p_data[data_len-1] == '\r')
        {
                while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint16_t index = 0;
        uint32_t ret_val;

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

                        do
                        {
                                ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                                if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                                {
                                        APP_ERROR_CHECK(ret_val);
                                }
                        } while (ret_val == NRF_ERROR_RESOURCES);

                        index = 0;
                }
                break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
                NRF_LOG_ERROR("Communication error occurred while handling UART.");
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
        ret_code_t err_code;

        switch (p_ble_nus_evt->evt_type)
        {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_INFO("Discovery complete.");
                err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
                APP_ERROR_CHECK(err_code);

                err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("Connected to device with Nordic UART Service.");
                break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
                ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
                break;

        case BLE_NUS_C_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected.");
                scan_start();
                break;
        }
}
/**@snippet [Handling events from the ble_nus_c module] */


static void ble_its_c_receive_cmd_data_handler(void)
{
        switch(m_its_rx_cmd_data.cmd_type)
        {
        case ITS_CMD_TX_TYPE_IMAGE_INFO:
        {
                NRF_LOG_INFO("Receive ITS_CMD_TX_TYPE_IMAGE_INFO");
                NRF_LOG_HEXDUMP_INFO(&m_its_rx_cmd_data, sizeof(m_its_rx_cmd_data));
                NRF_LOG_INFO("FileSize = 0x%04x, CRC32 0x%04x", m_its_rx_cmd_data.image_info.file_size_bytes, m_its_rx_cmd_data.image_info.crc32);
        }
        break;
        case ITS_CMD_TX_TYPE_DISCONNECT_REQ:
                break;
        case ITS_CMD_TX_TYPE_GET_FW_VERSION_REQ:
                break;
        case ITS_CMD_TX_TYPE_BATTERY_REQ:
                break;
        case ITS_CMD_TX_TYPE_LCD_STATUS_REQ:
                break;
        case ITS_CMD_TX_TYPE_WHITELIST_ENABLE:
                break;
        case ITS_CMD_TX_TYPE_WHITELIST_DISABLE:
                break;
        case ITS_CMD_TX_TYPE_WHITELIST_ADD:
                break;

        default:
                break;
        }
}


static void ble_its_c_evt_handler(ble_its_c_t *p_ble_its_c, ble_its_c_evt_t const *p_ble_its_c_evt)
{
        ret_code_t err_code;
        static uint32_t receive_byte = 0;

        switch (p_ble_its_c_evt->evt_type)
        {
        case BLE_ITS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_DEBUG("ITS Service: Discovery complete conn_handle = %d.", p_ble_its_c_evt->conn_handle);
                err_code = ble_its_c_handles_assign(p_ble_its_c, p_ble_its_c_evt->conn_handle, &p_ble_its_c_evt->handles);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ITS Notification is enabled!!");
                err_code = ble_its_c_tx_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ITS Image Notification is enabled !!!");
                err_code = ble_its_c_img_info_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_INFO("Connected to device with Nordic ITS Service.");
                break;

        case BLE_ITS_C_EVT_ITS_TX_EVT:
                receive_byte += p_ble_its_c_evt->data_len;
//                NRF_LOG_INFO("BLE_ITS_C_EVT_ITS_TX_EVT");
                NRF_LOG_INFO("Received Size = %x, Total %x", p_ble_its_c_evt->data_len, receive_byte);
//                NRF_LOG_INFO("%d", m_received_image_info.file_size_bytes);
                if (receive_byte == m_received_image_info.file_size_bytes)
                {
                        NRF_LOG_INFO("Data Received Completely");
                }

                break;

        case BLE_ITS_C_EVT_ITS_IMG_INFO_EVT:
        {
                //NRF_LOG_INFO("ITS Image Info: the number of bytes = %04d", receive_byte);
                memcpy(&m_its_rx_cmd_data, p_ble_its_c_evt->p_data,p_ble_its_c_evt->data_len);
                //app_sched_event_put(NULL, 0, ble_its_c_receive_cmd_data_handler);
                ble_its_c_receive_cmd_data_handler();
                // memcpy(&m_received_image_info, p_ble_its_c_evt->p_data, p_ble_its_c_evt->data_len);
                // NRF_LOG_INFO("BLE_ITS_C_EVT_ITS_IMG_INFO_EVT");
                // NRF_LOG_INFO("Data Size %x, CRC32 %04x", m_received_image_info.file_size_bytes, m_received_image_info.crc32);

                receive_byte = 0;
        }
        break;

        case BLE_ITS_C_EVT_ITS_RX_COMPLETE_EVT:

                NRF_LOG_INFO("Data is sent completely");
                NRF_LOG_INFO("BLE_ITS_C_EVT_ITS_RX_COMPLETE_EVT");
                break;

        case BLE_ITS_C_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected.");
                //scan_start();
                break;
        }
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        ret_code_t err_code;
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                             p_gap_evt->conn_handle);

                m_conn_handle = p_gap_evt->conn_handle;

                APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

                err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_its_c_handles_assign(&m_ble_its_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);

                // start discovery of services. The NUS Client waits for a discovery result
                err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
                APP_ERROR_CHECK(err_code);

                tx_power_connected_set();
                break;

        case BLE_GAP_EVT_DISCONNECTED:

                NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                m_conn_handle = BLE_CONN_HANDLE_INVALID;

                scan_start();

                break;

        case BLE_GAP_EVT_TIMEOUT:
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                        NRF_LOG_INFO("Connection Request timed out.");
                }
                break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported.
                err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
                // Accepting parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                        &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
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

        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        //ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = false; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
                              nrf_strerror_get(err_code));
        }

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint16_t data_length;
        if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
        {
                NRF_LOG_INFO("ATT MTU exchange completed.");
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_nus_max_data_len  = data_length;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
        else if (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED)
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_nus_max_data_len  = data_length;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);
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

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {

        case SEND_CMD_BUTTON_PIN:
                if (button_action == APP_BUTTON_PUSH)
                {

                }
                break;

        case SEND_DATA_BUTTON_PIN:
                if (button_action == APP_BUTTON_PUSH)
                {

                }
                break;
        case CHANGE_ADV_PAYLOAD_PIN:
                if (button_action == APP_BUTTON_PUSH)
                {

                }
        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {SEND_CMD_BUTTON_PIN, false, BUTTON_PULL, button_event_handler},
                {SEND_DATA_BUTTON_PIN, false, BUTTON_PULL, button_event_handler},
                {CHANGE_ADV_PAYLOAD_PIN, false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);
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

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
        ret_code_t err_code;
        ble_nus_c_init_t init;

        init.evt_handler = ble_nus_c_evt_handler;

        err_code = ble_nus_c_init(&m_ble_nus_c, &init);
        APP_ERROR_CHECK(err_code);
}


static void its_c_init(void)
{
        ret_code_t err_code;
        ble_its_c_init_t its_init;

        its_init.evt_handler = ble_its_c_evt_handler;
        err_code = ble_its_c_init(&m_ble_its_c, &its_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
        ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
                nrf_pwr_mgmt_run();
        }
}





static void load_device_setting(void)
{
        ret_code_t ret_code;

        // Check if not written
        uint8_t empty_addr[MAX_LEN_DEVICE_NAME];
        memset(empty_addr, 0xff, MAX_LEN_DEVICE_NAME);

        ble_gap_addr_t gap_addr;
        gap_addr.addr_id_peer = 0;
        gap_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

//        /* Read the Device ID (8 bytes) from UICR Memory */
//        uint8_t* ptr_device_id = (uint8_t*)&NRF_UICR->CUSTOMER[0];
//        if(memcmp(ptr_device_id, empty_addr, MAX_LEN_DEVICE_ID))
//        {
//                memcpy(m_manufacturer_specific_target_device_id, ptr_device_id, MAX_LEN_DEVICE_ID);
//                NRF_LOG_INFO("Device ID");
//                NRF_LOG_HEXDUMP_INFO(m_manufacturer_specific_target_device_id, MAX_LEN_DEVICE_ID);
//        }

        /* Read the Bluetooth Address from UICR Memory */
        // Get BD address from UICR memory
        uint8_t* ptr = (uint8_t*)&NRF_UICR->CUSTOMER[2];
        if(!memcmp(ptr, empty_addr, BLE_GAP_ADDR_LEN))
        {
                NRF_LOG_WARNING("No BD address in flash, using chip default");
                ret_code = sd_ble_gap_addr_get(&gap_addr);
                APP_ERROR_CHECK(ret_code);

                NRF_LOG_HEXDUMP_INFO(&gap_addr.addr, 6);

        }
        else
        {
                memcpy(&gap_addr.addr, ptr, BLE_GAP_ADDR_LEN);
                NRF_LOG_INFO("GAP Address");
                NRF_LOG_HEXDUMP_INFO(&gap_addr.addr, 6);
        }

//        /* Read the Device Name 12 bytes) from UICR Memory */
//        uint8_t* ptr_device_name = (uint8_t*)&NRF_UICR->CUSTOMER[4];
//        if(memcmp(ptr_device_name, empty_addr, MAX_LEN_DEVICE_NAME))
//        {
//                memcpy(m_device_name, ptr_device_name, MAX_LEN_DEVICE_NAME);
//                NRF_LOG_INFO("Device Name");
//                NRF_LOG_HEXDUMP_INFO(m_device_name, MAX_LEN_DEVICE_NAME);
//        }

        NRF_LOG_INFO("Setting bd address to: %02X:%02X:%02X:%02X:%02X:%02X",
                     gap_addr.addr[5],
                     gap_addr.addr[4],
                     gap_addr.addr[3],
                     gap_addr.addr[2],
                     gap_addr.addr[1],
                     gap_addr.addr[0]);

        ret_code = sd_ble_gap_addr_set(&gap_addr);
        APP_ERROR_CHECK(ret_code);
}


int main(void)
{
        // Initialize.
        log_init();
        timer_init();

        uart_init();
        buttons_init();
        db_discovery_init();
        power_management_init();
        ble_stack_init();

        load_device_setting();

        gatt_init();

        nus_c_init();
        its_c_init();
        scan_init();

        // Start execution.
        NRF_LOG_INFO("Central/Observer Example Start");
        scan_start();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}
