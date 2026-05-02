#include "common.h"
#include <sys/config.h>
//#include <time.h>
#include <sys/time.h>
#include "sysinit/sysinit.h"
#include "host/ble_gatt.h"
#include "services/cts/ble_svc_cts.h"

//#define TIME_ZONE_CHR
#define DEVICE_TIME_CHR (0x2A16)
static const char* TAG = "CTS";


static_assert(sizeof(time_t) == 4);
const struct ble_svc_cts_curr_time test = {
    .et_256 = {.d_d_t = { .d_t = {.year = 0x07EA, .month = 0x05, .day = 0x02, 
                            .hours = 0x0D, .minutes = 0x2A, .seconds = 0x1E
                        }, .day_of_week = 0x07 
                    }, .fractions_256 = 228 
        }, .adjust_reason = MANUAL_TIME_UPDATE_MASK
    };

struct timeval64 {
	uint64_t tv_sec; //gettimeofday use 64bit time_t
	uint32_t tv_usec;
};
//extern "C" {
//struct os_mbuf; struct ble_gatt_chr_def; struct ble_svc_cts_curr_time; struct tm;
//struct ble_svc_cts_local_time_info; struct ble_svc_cts_reference_time_info;
extern int ble_svc_cts_curr_time_validate(struct ble_svc_cts_curr_time curr_time);
extern int ble_svc_cts_local_time_info_validate(struct ble_svc_cts_local_time_info info);
//extern void ble_svc_cts_time_updated();
//}

/* characteristic values */
__unused static struct ble_svc_cts_local_time_info local_time_info_val = { .timezone = 4 * 3, .dst_offset = TIME_STANDARD };
static struct ble_svc_cts_curr_time current_local_time_val; static_assert(sizeof(current_local_time_val) == 10);
static struct ble_svc_cts_reference_time_info ref_time_info_val;

/* Characteristic value handles */
static uint16_t h_ble_svc_cts_curr_time;
__unused static uint16_t h_ble_svc_cts_local_time_info;
static uint16_t h_ble_svc_cts_ref_time;
static uint16_t h_ble_chr_device_time;

void set_cts_unix(time_t now);
time_t get_cts_unix() { return time(NULL); }
static void fetch_current_time(struct ble_svc_cts_curr_time *ctime);
static void set_current_time(struct ble_svc_cts_curr_time* ctime);
int fetch_local_time_info(struct ble_svc_cts_local_time_info *info) { return 0; }
int set_local_time_info(struct ble_svc_cts_local_time_info info) { return 0; }
static void fetch_reference_time_info(struct ble_svc_cts_reference_time_info *info);
/* Access function */
static int ble_svc_cts_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const ble_uuid16_t TAG_UUID16 = BLE_UUID16_INIT(BLE_SVC_CTS_UUID16);
static const ble_uuid16_t CHR_UUID16_CURRENT_TIME = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME);
__unused static const ble_uuid16_t CHR_UUID16_LOCAL_TIME_INFO = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO);
static const ble_uuid16_t CHR_UUID16_REF_TIME_INFO = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO);
static const ble_uuid16_t CHR_UUID16_DEVICE_TIME = BLE_UUID16_INIT(DEVICE_TIME_CHR);

static time_t last_updated;
static uint8_t adjust_reason;

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
	    }, 
#ifdef TIME_ZONE_CHR
		{ /*** Local info characteristic */
            .uuid = &CHR_UUID16_LOCAL_TIME_INFO.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,//BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_AUTHOR,
            .val_handle = &h_ble_svc_cts_local_time_info,
            
	    }, 
#endif
		{ /*** Reference time info Characteristic */
            .uuid = &CHR_UUID16_REF_TIME_INFO.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ,
            .val_handle = &h_ble_svc_cts_ref_time,
        },  {
            .uuid = &CHR_UUID16_DEVICE_TIME.u,
            .access_cb = ble_svc_cts_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,//BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_ENC,
            .val_handle = &h_ble_chr_device_time,
        } , { 0, /* No more characteristics in this service. */ } 
        },
    }, { 0, /* No more services. */ },
};

void gatt_cts_service_init() {
    CHECK_(ble_gatts_count_cfg(ble_svc_cts_defs));
    CHECK_(ble_gatts_add_svcs(ble_svc_cts_defs));
	setenv("TZ", "MSK-3", 1);
	tzset();
}

void set_cts_unix(time_t now) {
    struct timeval64 tv_now = {.tv_sec = now };
    settimeofday((struct timeval *)&tv_now, NULL);
    last_updated = now;//gettimeofday(&last_updated,/*  &tmz */NULL);  /* set the last updated */
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
    struct timeval64 tv_now = {.tv_sec = mktime(&timeinfo) };
    settimeofday((struct timeval *)&tv_now, NULL);
    last_updated = tv_now.tv_sec;  //gettimeofday(&last_updated, NULL);
    adjust_reason = ctime->adjust_reason; 
	ESP_LOGI(TAG, "%s tv_now %lu", __FUNCTION__, tv_now.tv_sec);
}

void fetch_current_time(struct ble_svc_cts_curr_time *ctime) {
	struct timeval64  val;
	struct timeval * const tv_now = (struct timeval *)&val;
	/* time given by 'time()' api does not persist after reboots */
	//time_t now = time(NULL);
	//localtime_r(&now, &timeinfo);
    gettimeofday(tv_now, NULL);
	struct tm *timeinfo = localtime(&tv_now->tv_sec); //&now //0x3fc95ed0 - 0x3fc94edc; 
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
        /* fractions_256 */ //tv_usec not bigger 1kk
        ctime->et_256.fractions_256 = (val.tv_usec * 256UL) / 1000000UL;
        ctime->adjust_reason = adjust_reason;
	ESP_LOGI(TAG, "%s %lu", __FUNCTION__, val.tv_usec);
}

void fetch_reference_time_info(struct ble_svc_cts_reference_time_info *info) {
    //struct timeval tv_now;
    //gettimeofday(&tv_now, NULL); /* ignore microseconds */
	time_t tv_sec = time(NULL) - last_updated; /* subtract the time when the last time was updated */
    info->time_source = TIME_SOURCE_MANUAL;//4
    info->time_accuracy = 0;
    uint32_t days_since_update = (tv_sec / 86400L);
    info->days_since_update = days_since_update < 255 ? days_since_update : 255;
    if(days_since_update <= 255) {
        uint32_t hours_since_update = (tv_sec % 86400L) / 3600;
        info->hours_since_update = hours_since_update;
    } else { info->hours_since_update = 255; }
    adjust_reason = (CHANGE_OF_DST_MASK | CHANGE_OF_TIME_ZONE_MASK);  
	ESP_LOGI(TAG, "%s last_updated %lu", __FUNCTION__, last_updated);
}

int ble_svc_cts_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    ESP_LOGD(TAG, "%s  conn_handle %u attr_handle %u",ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR ? "WRITE_CHR" :  
    ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR ? "READ_CHR"  : ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC ? "WRITE_DSC" :
    ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC ? "READ_DSC"  : "op?", conn_handle, attr_handle);
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    switch (uuid) {
    case BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME:
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            fetch_current_time(&current_local_time_val);
            CHECK_RET(os_mbuf_append(ctxt->om, &current_local_time_val, sizeof(current_local_time_val)));
			}
			return 0;// == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
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
#ifdef TIME_ZONE_CHR
    case BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO:
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: { ESP_LOGI(TAG, "get_local_tz");
            CHECK_RET(os_mbuf_append(ctxt->om, (void*)&local_time_info_val, sizeof(local_time_info_val)));
		} return 0;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: { ESP_LOGI(TAG, "set_local_tz");
            struct ble_svc_cts_local_time_info local_time_info = {};
			//rc = ble_svc_cts_chr_write(ctxt->om, sizeof(local_time_info), sizeof(local_time_info), &local_time_info, NULL);
            CHECK_RET(ble_hs_mbuf_to_flat(ctxt->om, &local_time_info, sizeof(local_time_info), NULL));
            CHECK_RET(ble_svc_cts_local_time_info_validate(local_time_info));
            /* just store the dst offset and timezone locally as we don't have the access to time using ntp server */
            local_time_info_val = local_time_info; /* gettimeofday(&last_updated, NULL); */ 
            /* set the adjust reason mask // notify the connected clients about the change in timezone and time */
            //current_local_time_val.adjust_reason |= (1 << 2) | (1 << 3);
			/* notify the connected clients about the change in timezone and time */
			ble_gatts_chr_updated(h_ble_svc_cts_curr_time);
            } return 0;
        }
		break;
#endif
    case BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO:
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            fetch_reference_time_info(&ref_time_info_val);
            CHECK_RET(os_mbuf_append(ctxt->om, &ref_time_info_val, sizeof(ref_time_info_val))); 
            return 0;
        } ESP_LOGW(TAG, "REF_TIME_INFO ctxt->op %u", ctxt->op); 
		break;

		//custom chr
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
    default: ESP_LOGW(TAG, "ctxt->chr->uuid: %u", uuid); //assert(0);
    }
    return BLE_ATT_ERR_UNLIKELY;
}
