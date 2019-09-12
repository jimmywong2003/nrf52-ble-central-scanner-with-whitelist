/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "ble_image_transfer_service.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ADVERTISING_INCLUDING_SCAN_RESPONSE_PAYLOAD 0

#define ADVERTISING_LED BSP_BOARD_LED_0 /**< Is on when device is advertising. */
#define CONNECTED_LED BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define DATA_TX_LED  BSP_BOARD_LED_2 /**< Is on when service is sending data. */
#define DATA_RX_LED  BSP_BOARD_LED_3 /**< Is on when service is receiving data */

#define SEND_IMAGE_INFO_BUTTON BSP_BUTTON_0   /**< Button that will trigger the notification event with the LED Button Service */
#define SEND_IMAGE_DATA_BUTTON BSP_BUTTON_1


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

/* Maximum Device Name in this example is 11 bytes */
#define DEVICE_NAME                     "DEVICE_0001"                              /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN+1               /**< UUID type for the Nordic UART Service (vendor specific). */
#define ITS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                 /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                32                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 20 ms). */

#define APP_ADV_UNLIMITED_PERIOD        0
#define APP_ADV_DURATION                APP_ADV_UNLIMITED_PERIOD                    /**< The advertising duration (180 seconds) in units of 10 milliseconds. */



#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 40 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define TX_POWER_LEVEL                  (0) /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                              /**< BLE NUS service instance. */
BLE_ITS_DEF(m_its, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                              /**< BLE IMAGE TRANSFER service instance. */

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle          = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
//        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
        {BLE_UUID_ITS_SERVICE, NUS_SERVICE_UUID_TYPE},
};

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static bool m_advertising_is_running = false;

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_srp_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];           /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
        .adv_data =
        {
                .p_data = m_enc_advdata,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
                .p_data = m_enc_srp_data,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

        }
};

#define COMPANY_IDENTIFIER              0x0059                                        /**< Company identifier for Nordic Semiconductor ASA as per www.bluetooth.org. */

#define ADV_ENCODED_AD_TYPE_LEN         1                                             /**< Length of encoded ad type in advertisement data. */
#define ADV_ENCODED_AD_TYPE_LEN_LEN     1                                             /**< Length of the 'length field' of each ad type in advertisement data. */
#define ADV_FLAGS_LEN                   1                                             /**< Length of flags field that will be placed in advertisement data. */
#define ADV_ENCODED_FLAGS_LEN           (ADV_ENCODED_AD_TYPE_LEN +       \
                                         ADV_ENCODED_AD_TYPE_LEN_LEN +   \
                                         ADV_FLAGS_LEN)                              /**< Length of flags field in advertisement packet. (1 byte for encoded ad type plus 1 byte for length of flags plus the length of the flags itself). */

#define ADV_ENCODED_COMPANY_ID_LEN      2                                            /**< Length of the encoded Company Identifier in the Manufacturer Specific Data part of the advertisement data. */

#define APP_ADVDATA_DEVICE_NAME         "DEVICE_00001"

#define APP_ADVDATA_DEVICE_NAME         0x44, 0x045, 0x56, 0x49, 0x43, 0x45, 0x5F, 0x030, 0x030, 0x030, 0x030, 0x031
//#define APP_ADVDATA_DEVICE_NAME 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C


#define APP_ADVDATA_MAC_ADDR    { 0xC0, 0x11, 0x22, 0x33, 0x44, 0xCC }

#define APP_ADVDATA_BATTERY_VALUE 0x64
#define APP_ADVDATA_TX_POWER      0x00
#define APP_ADVDATA_MANUFACTURER_OTHER_DATA 0x01, 0x02, 0x03, 0x04

#define APP_ADVDATA_DEVICE_NAME_LENGTH 0x0C
#define APP_ADVDATA_MAC_ADDR_LENGTH 0x06
#define APP_ADVDATA_BATTERY_LENGTH 0x01
#define APP_ADVDATA_TX_POWER_LENGTH 0x01
#define APP_ADVDATA_OTHER_LENGTH 0x06

#define ADV_ADDL_MANUF_DATA_LEN         (BLE_GAP_ADV_SET_DATA_SIZE_MAX -                \
                                         (                                     \
                                                 ADV_ENCODED_FLAGS_LEN +           \
                                                 (                                 \
                                                         ADV_ENCODED_AD_TYPE_LEN +     \
                                                         ADV_ENCODED_AD_TYPE_LEN_LEN + \
                                                         ADV_ENCODED_COMPANY_ID_LEN    \
                                                 )                                 \
                                         )                                     \
                                         )

/**< Value of the additional manufacturer specific data that will be placed in air (initialized to all zeros). */
static uint8_t m_addl_adv_manuf_data[ADV_ADDL_MANUF_DATA_LEN]  =
{
        APP_ADVDATA_DEVICE_NAME,                                 /* 6 bytes for Device ID */
        APP_ADVDATA_MAC_ADDR,                                  /* 6 bytes for BLE MAC Address */
        APP_ADVDATA_BATTERY_VALUE,                             /* 1 byte for battery value in % */
        APP_ADVDATA_TX_POWER,                                /* 1 byte for the measure RSSI value */
        APP_ADVDATA_MANUFACTURER_OTHER_DATA,                                /* 8 bytes for other data*/
};

static void advertising_start(void);


#define CHANGE_ADV_PAYLOAD_INTERVAL     APP_TIMER_TICKS(2000)                 /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */
APP_TIMER_DEF(m_change_adv_payload_timer_id);                      /**< Battery measurement timer. */

static uint8_t adv_count = 0;

static void change_adv_payload_timeout_handler(void * p_context)
{
        UNUSED_PARAMETER(p_context);
        ret_code_t err_code;
        ble_advdata_t advdata;

        ble_advdata_manuf_data_t manuf_data;
        manuf_data.company_identifier = COMPANY_IDENTIFIER;
        manuf_data.data.size          = ADV_ADDL_MANUF_DATA_LEN;
        manuf_data.data.p_data        = m_addl_adv_manuf_data;
        advdata.p_manuf_specific_data = &manuf_data;

        sprintf(m_addl_adv_manuf_data, "DEVICE_%05d", adv_count);
        adv_count++;

        memcpy(m_adv_data.adv_data.p_data, m_addl_adv_manuf_data, 0x1B);
//
//        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
//        APP_ERROR_CHECK(err_code);



        NRF_LOG_HEXDUMP_INFO(m_adv_data.adv_data.p_data, m_adv_data.adv_data.len);

//        err_code = sd_ble_gap_adv_set_configure(m_adv_data.adv_data.p_data, m_adv_data.adv_data.len, NULL, 0);
//        APP_ERROR_CHECK(err_code);


}

static void change_adv_payload_start(void)
{

        ret_code_t err_code;

        err_code = app_timer_create(&m_change_adv_payload_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    change_adv_payload_timeout_handler);
        APP_ERROR_CHECK(err_code);

        err_code = app_timer_start(m_change_adv_payload_timer_id, CHANGE_ADV_PAYLOAD_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);
}

#if LEDS_NUMBER > 0
static const uint8_t m_board_led_list[LEDS_NUMBER] = LEDS_LIST;
#endif

#if BUTTONS_NUMBER > 0
static const uint8_t m_board_btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
#endif

static void bsp_board_leds_init(void)
{
    #if defined(BOARD_PCA10059)
        // If nRF52 USB Dongle is powered from USB (high voltage mode),
        // GPIO output voltage is set to 1.8 V by default, which is not
        // enough to turn on green and blue LEDs. Therefore, GPIO voltage
        // needs to be increased to 3.0 V by configuring the UICR register.
        if (NRF_POWER->MAINREGSTATUS &
            (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos))
        {
                gpio_output_voltage_setup();
        }
    #endif

        uint32_t i;
        for (i = 0; i < LEDS_NUMBER; ++i)
        {
                nrf_gpio_cfg_output(m_board_led_list[i]);
        }
        bsp_board_leds_off();
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
        uint32_t err_code;
        ble_gap_conn_params_t gap_conn_params;
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *) DEVICE_NAME,
                                              strlen(DEVICE_NAME));
        APP_ERROR_CHECK(err_code);

        memset(&gap_conn_params, 0, sizeof(gap_conn_params));

        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
        gap_conn_params.slave_latency     = SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

        if (p_evt->type == BLE_NUS_EVT_RX_DATA)
        {
                uint32_t err_code;

                NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
                NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

                for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
                {
                        do
                        {
                                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                                {
                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                        APP_ERROR_CHECK(err_code);
                                }
                        } while (err_code == NRF_ERROR_BUSY);
                }
                if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
                {
                        while (app_uart_put('\n') == NRF_ERROR_BUSY);
                }
        }

}
/**@snippet [Handling the data received over BLE] */


static void its_evt_handler(ble_its_t *p_its, ble_its_evt_t const *p_its_evt)
{
        switch (p_its_evt->evt_type)
        {
        case BLE_ITS_EVT_ITS_RX_CMD_EVT:
        {
                NRF_LOG_INFO("BLE_ITS_EVT_ITS_RX_CMD_EVT");
        }
        break;

        case BLE_ITS_EVT_ITS_RX_DATA_EVT:
                NRF_LOG_INFO("BLE_ITS_EVT_ITS_TX_DATA_COMPLETE_EVT");
                break;

        case BLE_ITS_EVT_ITS_TX_DATA_COMPLETE_EVT:
                NRF_LOG_INFO("BLE_ITS_EVT_ITS_TX_DATA_COMPLETE_EVT");
                break;

        case BLE_ITS_EVT_ITS_TX_DATA_READY_EVT:
                NRF_LOG_INFO("BLE_ITS_EVT_ITS_TX_DATA_READY_EVT");
                break;

        case BLE_ITS_EVT_ITS_TX_INFO_READY_EVT:
                NRF_LOG_INFO("BLE_ITS_EVT_ITS_TX_INFO_READY_EVT");
                break;

        case BLE_ITS_EVT_DISCONNECTED:
                NRF_LOG_INFO("BLE_ITS_EVT_DISCONNECTED");
                break;

        case BLE_ITS_EVT_CONNECTED:
                NRF_LOG_INFO("BLE_ITS_EVT_CONNECTED");
                break;

        default:
                break;
        }
}


static void nus_int(void)
{

        uint32_t err_code;
        ble_nus_init_t nus_init;

        // Initialize NUS.
        memset(&nus_init, 0, sizeof(nus_init));

        nus_init.data_handler = nus_data_handler;

        err_code = ble_nus_init(&m_nus, &nus_init);
        APP_ERROR_CHECK(err_code);

}

static void image_transfer_services_init(void)
{

        uint32_t err_code;
        ble_its_init_t its_init;

        memset(&its_init, 0, sizeof(its_init));
        // Initialize ITS.
        its_init.evt_handler = its_evt_handler;
        err_code = ble_its_init(&m_its, &its_init);
        APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        uint32_t err_code;

        nrf_ble_qwr_init_t qwr_init = {0};

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);

        /* Nordic UART Service */
        nus_int();

        /* Nordic Image Transfer Service */
        image_transfer_services_init();
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
        uint32_t err_code;

        if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
                APP_ERROR_CHECK(err_code);
        }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
        uint32_t err_code;
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = on_conn_params_evt;
        cp_init.error_handler                  = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
}

#if 0
/**@defgroup BLE_GAP_ADDR_TYPES GAP Address types
 * @{ */
#define BLE_GAP_ADDR_TYPE_PUBLIC                        0x00 /**< Public (identity) address.*/
#define BLE_GAP_ADDR_TYPE_RANDOM_STATIC                 0x01 /**< Random static (identity) address. */
#define BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE     0x02 /**< Random private resolvable address. */
#define BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE 0x03 /**< Random private non-resolvable address. */
#define BLE_GAP_ADDR_TYPE_ANONYMOUS                     0x7F /**< An advertiser may advertise without its address.
                                                                  This type of advertising is called anonymous. */
/**@} */
#endif

static void Get_Connect_MAC_Address(ble_gap_addr_t *gap_address)
{
        NRF_LOG_INFO("Connected to Central Address to: %02X:%02X:%02X:%02X:%02X:%02X",
                     gap_address->addr[5],
                     gap_address->addr[4],
                     gap_address->addr[3],
                     gap_address->addr[2],
                     gap_address->addr[1],
                     gap_address->addr[0]);

        switch (gap_address->addr_type)
        {
        case BLE_GAP_ADDR_TYPE_PUBLIC:
                NRF_LOG_INFO("Central BLE Address Type : BLE_GAP_ADDR_TYPE_PUBLIC");
                break;
        case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
                NRF_LOG_INFO("Central BLE Address Type : BLE_GAP_ADDR_TYPE_RANDOM_STATIC");
                break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
                NRF_LOG_INFO("Central BLE Address Type : BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE");
                break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
                NRF_LOG_INFO("Central BLE Address Type : BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE");
                break;
        default:
                NRF_LOG_INFO("Other type of address");

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
        uint32_t err_code;

        // For readability.
        uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        uint16_t role        = ble_conn_state_role(conn_handle);
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:

                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
                APP_ERROR_CHECK(err_code);
                Get_Connect_MAC_Address((ble_gap_addr_t *)&p_gap_evt->params.connected.peer_addr);

                m_advertising_is_running = false;
                bsp_board_led_off(ADVERTISING_LED);
                bsp_board_led_on(CONNECTED_LED);
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);
                // LED indication will be changed when advertising starts.
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                advertising_start();
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

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported
                err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        default:
                // No implementation needed.
                break;
        }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
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
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        //ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = true; /* Enable channel survey role */

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
        uint8_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_nus_max_data_len = data_length;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                m_ble_nus_max_data_len = data_length;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
        }
        NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
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
        case SEND_IMAGE_INFO_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("Send button state change.");
                }
                break;

        case SEND_IMAGE_DATA_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("Send button state change.");
                }
                break;

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
                {SEND_IMAGE_INFO_BUTTON, false, BUTTON_PULL, button_event_handler},
                {SEND_IMAGE_DATA_BUTTON, false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);
}





/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint8_t index = 0;
        uint32_t err_code;

        switch (p_event->evt_type)
        {
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= m_ble_nus_max_data_len))
                {
                        if (index > 1)
                        {
                                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                                do
                                {
                                        uint16_t length = (uint16_t)index;
                                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                            (err_code != NRF_ERROR_RESOURCES) &&
                                            (err_code != NRF_ERROR_NOT_FOUND))
                                        {
                                                APP_ERROR_CHECK(err_code);
                                        }
                                } while (err_code == NRF_ERROR_RESOURCES);
                        }

                        index = 0;
                }
                break;

        case APP_UART_COMMUNICATION_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{

        ret_code_t err_code;
        ble_advdata_t advdata;
        ble_advdata_t srdata;
        ble_advdata_manuf_data_t manuf_data;

        memset(&advdata, 0, sizeof(advdata));

        advdata.include_appearance = false;
        advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        manuf_data.company_identifier = COMPANY_IDENTIFIER;
        manuf_data.data.size          = ADV_ADDL_MANUF_DATA_LEN;
        manuf_data.data.p_data        = m_addl_adv_manuf_data;
        advdata.p_manuf_specific_data = &manuf_data;

        // Build and set advertising data.
        memset(&srdata, 0, sizeof(srdata));

#ifdef ADVERTISING_INCLUDING_SCAN_RESPONSE_PAYLOAD

        srdata.name_type = BLE_ADVDATA_FULL_NAME;
        srdata.include_appearance = false;
        srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        srdata.uuids_complete.p_uuids = m_adv_uuids;

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);

        ble_gap_adv_params_t adv_params;

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));

        NRF_LOG_HEXDUMP_INFO(m_adv_data.adv_data.p_data, m_adv_data.adv_data.len);

        adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
        adv_params.duration = APP_ADV_DURATION;
        adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        adv_params.p_peer_addr = NULL;
        adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
        adv_params.interval = APP_ADV_INTERVAL;
#else

        sprintf(&m_addl_adv_manuf_data[0], " %s", (char *)DEVICE_NAME);

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);

        ble_gap_adv_params_t adv_params;

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));

        adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
        adv_params.duration = APP_ADV_DURATION;
        adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        //adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
        adv_params.p_peer_addr = NULL;
        adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
        adv_params.interval = APP_ADV_INTERVAL;

#endif
        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
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


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
                nrf_pwr_mgmt_run();
        }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{

        if (m_advertising_is_running != true)
        {
                ret_code_t err_code;
                err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
                APP_ERROR_CHECK(err_code);
                m_advertising_is_running = true;
        }
        bsp_board_led_on(ADVERTISING_LED);
        bsp_board_led_off(CONNECTED_LED);

}

static void load_device_setting(void)
{
        ret_code_t ret_code;

        // Check if not written
        uint8_t empty_addr[BLE_GAP_ADDR_LEN];
        memset(empty_addr, 0xff, BLE_GAP_ADDR_LEN);

        ble_gap_addr_t gap_addr;
        gap_addr.addr_id_peer = 0;
        gap_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;

        /* Read the Bluetooth Address from UICR Memory */
        // Get BD address from UICR memory
        uint8_t* ptr = (uint8_t*)&NRF_UICR->CUSTOMER[2];
        if(!memcmp(ptr, empty_addr, BLE_GAP_ADDR_LEN))
        {
                NRF_LOG_WARNING("No BD address in flash, using chip default");
                ret_code = sd_ble_gap_addr_get(&gap_addr);
                APP_ERROR_CHECK(ret_code);
                gap_addr.addr[5] = 0xCC;
                gap_addr.addr[4] = 0xCC;
                gap_addr.addr[3] = 0xCC;
                gap_addr.addr[2] = 0xCC;
                gap_addr.addr[1] = 0xCC;
                gap_addr.addr[0] = 0xCC;
                NRF_LOG_HEXDUMP_DEBUG(&gap_addr.addr, 6);
        }
        else
        {
                memcpy(&gap_addr.addr, ptr, BLE_GAP_ADDR_LEN);
                NRF_LOG_INFO("GAP Address");
                NRF_LOG_HEXDUMP_DEBUG(&gap_addr.addr, 6);
        }

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

/**@brief Application main function.
 */
int main(void)
{
        bool erase_bonds;

        // Initialize.
        uart_init();
        log_init();
        timers_init();
        bsp_board_leds_init();
        buttons_init();

        power_management_init();
        ble_stack_init();
        scheduler_init();

        load_device_setting();
        gap_params_init();
        gatt_init();
        services_init();
        advertising_init();
        conn_params_init();

        // Start execution.
        NRF_LOG_INFO("Device Name %s in Advertising", (char *)DEVICE_NAME);
        advertising_start();

        // change the advertising payload
        change_adv_payload_start();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}


/**
 * @}
 */
