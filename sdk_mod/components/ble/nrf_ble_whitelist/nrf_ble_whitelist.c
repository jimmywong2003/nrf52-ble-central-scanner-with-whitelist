/**
 * Copyright (c) 2018 - 2019, Nordic Semiconductor ASA
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

#include "sdk_common.h"

#include "sdk_config.h"
#include <stdlib.h>

#if 1//NRF_MODULE_ENABLED(NRF_BLE_WHITELIST)

#include <string.h>
#include "app_error.h"
#include "nrf_assert.h"
#include "sdk_macros.h"
#include "ble_advdata.h"

#include "nrf_ble_whitelist.h"

/* This module is working only on the random static address / public address */

static bool m_whitelist_is_running = false;

static ble_gap_addr_t const * m_whitelist_addr_ptrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
static ble_gap_addr_t m_whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
static uint8_t m_addr_cnt = 0;

ret_code_t nrf_ble_whitelist_add(ble_gap_addr_t *addr, uint8_t * whitelist_count)
{

        if (m_addr_cnt >= BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                return NRF_ERROR_DATA_SIZE;

        for (uint32_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
        {
                if (memcmp(&m_whitelist_addrs[i], addr, sizeof(ble_gap_addr_t))==0)
                {
                        //NRF_LOG_INFO("[Whitelist]: Duplicate Whitelist add!");
                        return NRF_ERROR_INVALID_PARAM;
                }
        }

        memcpy(&m_whitelist_addrs[m_addr_cnt], addr, sizeof(ble_gap_addr_t));

        m_addr_cnt++;

        //return the number of whitelist store
        *whitelist_count = m_addr_cnt;

        return NRF_SUCCESS;
}

ret_code_t nrf_ble_whitelist_enable(void)
{
        ret_code_t ret;

        m_whitelist_is_running = true;

        if (m_addr_cnt == 0)
        {
                return NRF_ERROR_DATA_SIZE;
        }

        for (uint32_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
        {
                m_whitelist_addr_ptrs[i] = &m_whitelist_addrs[i];
        }

        ret = sd_ble_gap_whitelist_set(m_whitelist_addr_ptrs, m_addr_cnt);
        APP_ERROR_CHECK(ret);

        return NRF_SUCCESS;
}


uint8_t nrf_ble_whitelist_cnt(void)
{
        return m_addr_cnt;
}

ret_code_t nrf_ble_whitelist_clear(void)
{
        ret_code_t ret;
        memset(m_whitelist_addrs, 0, sizeof(m_whitelist_addrs));
        m_addr_cnt = 0;

        ret = sd_ble_gap_whitelist_set(NULL, 0);
        APP_ERROR_CHECK(ret);


        m_whitelist_is_running = false;
        return ret;
}


bool nrf_ble_whitelist_is_running(void)
{
        return m_whitelist_is_running;
}



#endif // NRF_BLE_WHITELIST_ENABLED
