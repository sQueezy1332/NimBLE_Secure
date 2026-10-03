#pragma once
/* STD APIs */
#define _WANT_USE_LONG_TIME_T
#include "esp_check.h"

#include "host/ble_hs.h"
//#include "nimble/ble.h"
//#include "modlog/modlog.h"

#if !CONFIG_BT_NIMBLE_LOG_LEVEL_NONE
#define DEBUG_LOG
#define NIMLOG(msg, ...) printf(msg, ##__VA_ARGS__)
#define SCAN_PASSIVE 0 
#else
#define NIMLOG(msg, ...) 
#define SCAN_PASSIVE 1
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define FUNC_ADDRESS esp_cpu_get_call_addr((intptr_t)__builtin_return_address(0))
#define CHECK_(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x)
#define CHECK_RET(x) ESP_RETURN_ON_ERROR(x,"","err 0x%02x at 0x%08x", err_rc_, FUNC_ADDRESS)
#define CHECK_VOID(x) ESP_RETURN_VOID_ON_ERROR(x,"","err 0x%02x at 0x%08x", err_rc_, FUNC_ADDRESS)