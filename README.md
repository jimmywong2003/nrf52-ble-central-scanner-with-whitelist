# nrf52-ble-central-whitelist

Example how to add the BLE Central with Whitelist 

# Description

This example is to show how to add the whitelist module on the ble central scanning in order to improve the target device time.

# Compile switch on the BLE Observer / Central
 
```
#define SCAN_WITH_WHITELIST_ENABLED 1           /**< Scanning with Whitelist .*/

#define NRF_BLE_SCAN_ACTIVE_SCANNING 0          /**< 0 -- passive scanning, 1 -- active scanning. */

#define SCAN_MATCH_WITH_CONNECT                 /**< Connect with Match */
```

## Requirement
* NRF52840 DK x 2
* SDK 15.3 / S140v6.1.1
* Segger Embedded Studio
