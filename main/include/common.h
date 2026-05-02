#pragma once
/* STD APIs */
#define _WANT_USE_LONG_TIME_T
//#include <assert.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include <string.h>

/* ESP APIs */
//#include "esp_random.h"
//#include "nvs_flash.h"
#include "esp_check.h"

/* FreeRTOS APIs */
//#include <freertos/FreeRTOS.h>
//#include <freertos/task.h>

/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
//#include "nimble/ble.h"
#include "modlog/modlog.h"

#if _ESP_LOG_ENABLED(3)
#define NIMLOG(msg, ...) esp_rom_printf((msg), ##__VA_ARGS__)
#define SCAN_PASSIVE 0 
#else
#define NIMLOG(msg, ...) 
#define SCAN_PASSIVE 1
#endif

#define FUNC_ADDRESS (esp_cpu_get_call_addr((intptr_t)__builtin_return_address(0)))
#define CHECK_(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x)
#define CHECK_RET(x) ESP_RETURN_ON_ERROR(x,"","0x%08x",FUNC_ADDRESS)
#define CHECK_VOID(x) ESP_RETURN_VOID_ON_ERROR(x,"","0x%08x",FUNC_ADDRESS)
