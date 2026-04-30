#define _WANT_USE_LONG_TIME_T
#include <sys/config.h>
#include <sys/time.h>
#include "sysinit/sysinit.h"
#include "host/ble_gatt.h"
#include "services/cts/ble_svc_cts.h"

//#include "syscfg/syscfg.h"
//#include "host/ble_hs_mbuf.h"

#include "common.h"
#define DEVICE_TIME_CHR (0x2A16)
static const char* TAG = "CTS";

static_assert(sizeof(time_t) == 4);
// const struct ble_svc_cts_curr_time test = {
//     .et_256 {.d_d_t {.d_t  {.year = 0x07E9, .month = 0x0B, .day = 0x17, 
//                             .hours = 0x0D, .minutes = 0x2A, .seconds = 0x1E
//                         }, .day_of_week = 0x07 
//                     }, .fractions_256 = 0 
//         }, .adjust_reason = MANUAL_TIME_UPDATE_MASK
//     };
//extern "C" {
//struct os_mbuf; struct ble_gatt_chr_def; struct ble_svc_cts_curr_time; struct tm;
//struct ble_svc_cts_local_time_info; struct ble_svc_cts_reference_time_info;
extern int ble_svc_cts_curr_time_validate(struct ble_svc_cts_curr_time curr_time);
extern int ble_svc_cts_local_time_info_validate(struct ble_svc_cts_local_time_info info);
//extern void ble_svc_cts_time_updated();
//}

const ble_uuid16_t TAG_UUID16 = BLE_UUID16_INIT(BLE_SVC_CTS_UUID16);
const ble_uuid16_t CHR_UUID16_CURRENT_TIME = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME);
const ble_uuid16_t CHR_UUID16_LOCAL_TIME_INFO = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO);
const ble_uuid16_t CHR_UUID16_REF_TIME_INFO = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO);
const ble_uuid16_t CHR_UUID16_DEVICE_TIME = BLE_UUID16_INIT(DEVICE_TIME_CHR);

void set_cts_unix(time_t now);
time_t get_cts_unix() { return time(NULL); }
static void set_current_time(struct ble_svc_cts_curr_time* ctime);
static void fetch_current_time(struct ble_svc_cts_curr_time *ctime);
static void fetch_reference_time_info(struct ble_svc_cts_reference_time_info *info);
static int ble_svc_cts_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static struct ble_svc_cts_local_time_info local_time_info = { .timezone = 4 * 3, .dst_offset = TIME_STANDARD };
static struct ble_svc_cts_curr_time current_local_time_val;
static struct ble_svc_cts_reference_time_info ref_time_info_val;
static struct timeval last_updated;
static uint8_t adjust_reason;

/* Characteristic value handles */
static uint16_t h_ble_svc_cts_curr_time;
static uint16_t h_ble_svc_cts_local_time_info;
static uint16_t h_ble_svc_cts_ref_time;
static uint16_t h_ble_chr_device_time;

static const struct ble_gatt_svc_def ble_svc_cts_defs[] = { 
    {       /*** Current Time Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &TAG_UUID16.u,
        .characteristics = (struct ble_gatt_chr_def[]) { 
        { /*** Current Time characteristic */
            .uuid = &CHR_UUID16_CURRENT_TIME.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN | BLE_GATT_CHR_F_NOTIFY,
	        .val_handle = &h_ble_svc_cts_curr_time,
	    }, { /*** Local info characteristic */
            .uuid = &CHR_UUID16_LOCAL_TIME_INFO.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,//BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_AUTHOR,
            .val_handle = &h_ble_svc_cts_local_time_info,
            
	    }, { /*** Reference time info Characteristic */
            .uuid = &CHR_UUID16_REF_TIME_INFO.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ,
            .val_handle = &h_ble_svc_cts_ref_time,
        },  {
            .uuid = &CHR_UUID16_DEVICE_TIME.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,//BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_ENC,
            .val_handle = &h_ble_chr_device_time,
        } ,  { 0, /* No more characteristics in this service. */ } 
        },
    }, { 0, /* No more services. */ },
};

void set_cts_unix(time_t now) {
    //timezone tmz = { .tz_minuteswest = local_time_info.timezone / 4 *60,  };
    struct timeval tv_now = {.tv_sec = now };
    settimeofday(&tv_now, /* &tmz */NULL);
    gettimeofday(&last_updated,/*  &tmz */NULL);  /* set the last updated */
    adjust_reason = MANUAL_TIME_UPDATE_MASK; 
	ESP_LOGI(TAG, "set_cts_unix %lu", now);
}

void set_current_time(struct ble_svc_cts_curr_time *ctime) {
    struct tm timeinfo = {
        .tm_sec = ctime->et_256.d_d_t.d_t.seconds,
        .tm_min = ctime->et_256.d_d_t.d_t.minutes,
        .tm_hour = ctime->et_256.d_d_t.d_t.hours,
        .tm_mday = ctime->et_256.d_d_t.d_t.day,
        .tm_mon = ctime->et_256.d_d_t.d_t.month - 1,
        .tm_year= ctime->et_256.d_d_t.d_t.year - 1900,
        .tm_wday = ctime->et_256.d_d_t.day_of_week - 1,
    };
    struct timeval tv_now = {.tv_sec = mktime(&timeinfo),};
    //timezone tmz = { .tz_minuteswest = local_time_info.timezone / 4 * 60,  };
    settimeofday(&tv_now, /* &tmz */NULL);
    gettimeofday(&last_updated, /* &tmz */NULL);
    adjust_reason = ctime->adjust_reason; 
	ESP_LOGI(TAG, "%s tv_now %lu", __FUNCTION__, tv_now.tv_sec);
}

void fetch_current_time(struct ble_svc_cts_curr_time *ctime) {
    struct timeval tv_now; //timezone tmz = { .tz_minuteswest = local_time_info.timezone / 4 *60,  };
    //time_t now = time(NULL); 
    gettimeofday(&tv_now, /* &tmz */NULL);
    struct tm *timeinfo = localtime(&tv_now.tv_sec); //&now
    //localtime_r(&now, &timeinfo); //localtime
    //if(ctime) {
        /* fill date_time */
        ctime->et_256.d_d_t.d_t.year = timeinfo->tm_year + 1900;
        ctime->et_256.d_d_t.d_t.month = timeinfo->tm_mon + 1;
        ctime->et_256.d_d_t.d_t.day = timeinfo->tm_mday;
        ctime->et_256.d_d_t.d_t.hours = timeinfo->tm_hour;
        ctime->et_256.d_d_t.d_t.minutes = timeinfo->tm_min;
        ctime->et_256.d_d_t.d_t.seconds = timeinfo->tm_sec;
        /* day of week time gives day range of [0, 6], current_time_sevice
           has day range of [1,7] */ //1 jan 1970 is thursday
        ctime->et_256.d_d_t.day_of_week = timeinfo->tm_wday  /* + 1 */; //friday
        /* fractions_256 */
        ctime->et_256.fractions_256 = (((uint64_t)tv_now.tv_usec * 256UL )/ 1000000L);
        ctime->adjust_reason = adjust_reason;
    //}
	ESP_LOGI(TAG, "%s %lu",tv_now.tv_usec, __FUNCTION__);
}

void fetch_reference_time_info(struct ble_svc_cts_reference_time_info *info) {
    struct timeval tv_now;
    uint64_t days_since_update, hours_since_update;
    gettimeofday(&tv_now, NULL);
    /* subtract the time when the last time was updated */
    tv_now.tv_sec -= last_updated.tv_sec; /* ignore microseconds */
    info->time_source = TIME_SOURCE_MANUAL;
    info->time_accuracy = 0;
    days_since_update = (tv_now.tv_sec / 86400L);
    hours_since_update = (tv_now.tv_sec / 3600);
    info->days_since_update = days_since_update < 255 ? days_since_update : 255;
    if(days_since_update > 254) { info->hours_since_update = 255; }
    else {
        hours_since_update = (tv_now.tv_sec % 86400L) / 3600;
        info->hours_since_update = hours_since_update;
    }
    adjust_reason = (CHANGE_OF_DST_MASK | CHANGE_OF_TIME_ZONE_MASK);  
	ESP_LOGI(TAG, "%s last_updated %lu", __FUNCTION__, last_updated.tv_sec);
}

int ble_svc_cts_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    ESP_LOGI(TAG, "%s  conn_handle %u attr_handle %u",ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR ? "WRITE_CHR" :  
    ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR ? "READ_CHR"  : ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC ? "WRITE_DSC" :
    ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC ? "READ_DSC"  : "op?", conn_handle, attr_handle);;
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    switch (uuid) {
    case BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME:
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            fetch_current_time(&current_local_time_val);
            CHECK_RET(os_mbuf_append(ctxt->om, &current_local_time_val, sizeof(current_local_time_val)));
			} return 0;// == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            struct ble_svc_cts_curr_time curr_time = {};
            CHECK_RET(ble_hs_mbuf_to_flat(ctxt->om, &curr_time, sizeof(curr_time), NULL));
            CHECK_RET(ble_svc_cts_curr_time_validate(curr_time));
            set_current_time(&curr_time); 
			current_local_time_val = curr_time;
            ble_gatts_chr_updated(attr_handle); /* schedule notifications for subscribed peers */
            } return 0;
        }
       	break;
    case BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO:
        //assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: { ESP_LOGI(TAG, "get_local_tz");
            CHECK_RET(os_mbuf_append(ctxt->om, (void*)&local_time_info, sizeof(local_time_info)));
		} return 0;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: { 
            struct ble_svc_cts_local_time_info time_info = {};
            CHECK_RET(ble_hs_mbuf_to_flat(ctxt->om, &time_info, sizeof(time_info), NULL));
            CHECK_RET(ble_svc_cts_local_time_info_validate(time_info));
            /* just store the dst offset and timezone locally as we don't have the access to time using ntp server */
            local_time_info = time_info; /* gettimeofday(&last_updated, NULL); */ ESP_LOGI(TAG, "set_local_tz");
                /* set the adjust reason mask // notify the connected clients about the change in timezone and time */
                ble_gatts_chr_updated(h_ble_svc_cts_curr_time);
            } return 0;
        }
		break;
    case BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO:
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            fetch_reference_time_info(&ref_time_info_val);
            CHECK_RET(os_mbuf_append(ctxt->om, &ref_time_info_val, sizeof(ref_time_info_val))); 
            return 0;
        } ESP_LOGW(TAG, "REF_TIME_INFO ctxt->op %u", ctxt->op); 
		break;
    case DEVICE_TIME_CHR:  
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            time_t val = get_cts_unix();
            CHECK_RET(os_mbuf_append(ctxt->om, &val, sizeof(val)));
        } return 0;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            if(ctxt->om->om_len != sizeof(time_t)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            set_cts_unix(*(time_t*)ctxt->om->om_data);
			} return 0;
        }
        break;
    default: ESP_LOGW(TAG, "ctxt->chr->uuid: %u", uuid);
        //assert(0);
    }
    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_cts_service_init() {
    CHECK_(ble_gatts_count_cfg(ble_svc_cts_defs));
    CHECK_(ble_gatts_add_svcs(ble_svc_cts_defs));
}
