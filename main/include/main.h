#pragma once
#include "ESP_MAIN.h"
#include "esp_random.h"
#include "mbedtls/md.h"
#include "rom/crc.h"
#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "heart_rate.h"
#include "led.h"
#include "credentials.h"
#define PIN_LED_D4 12
#define TIMER_CHANGE_NAME pdMS_TO_TICKS(1000*60*60 * 1)
#define TIMER_PATCH pdMS_TO_TICKS(1000 * 5) //TIMER_CHANGE_NAME
#define KALMAN_KOEF 0.5

static void nimble_host_config_init();
static void nimble_host_task(void *);

static void heart_rate_task(void *);
extern void periodic_sync_scan();

static void rand_device_name();
static uint32_t generate_pin(const char *, const char *);
void create_hex_string(char*, cbyte* , byte);
void change_device_name();
int ble_delete_all_peers();
uint32_t Kalman(uint32_t);

static char password[] = DEFAULT_PASS; //static_assert(sizeof(password)-1 == 8);
static uint32_t pincode;
static bool sensor_state;
static bool synced = false;

StaticTimer_t  xTimerPatchBuffer;
TimerHandle_t timerPatch;

uint32_t get_pincode() { 
#ifdef DEBUG_ENABLE
    return 111111;
#else
    return pincode; 
#endif
}

void patch_func() {
    digitalWrite(PIN_LED_D4, sensor_state = 1); //return false;
    if(xTimerReset(timerPatch, pdMS_TO_TICKS(100))) {  /* return true; */}
}

static void get_task_list(String& str);
static String get_task_list() { String str; get_task_list(str); return str; }
void print_task_list() { DEBUGLN(get_task_list()); };
#ifdef __cplusplus
extern "C" {
#endif
extern void ble_store_config_init(void);
#ifdef __cplusplus
}
#endif