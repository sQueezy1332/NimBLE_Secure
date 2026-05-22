#include "common.h"
#include <sys/config.h>
//#include <time.h>
#include <sys/time.h>
#include "sysinit/sysinit.h"
#include "host/ble_gatt.h"
#include "services/cts/ble_svc_cts.h"

//#define TIME_ZONE_CHR
//#define CURRENT_TIME_WRITE
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
static const ble_uuid16_t SVC_CTS = BLE_UUID16_INIT(BLE_SVC_CTS_UUID16);
static const ble_uuid16_t CHR_CURRENT_TIME = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME);
__unused static const ble_uuid16_t CHR_TIME_ZONE = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO);
static const ble_uuid16_t CHR_REF_TIME_INFO = BLE_UUID16_INIT(BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO);
static const ble_uuid16_t CHR_DEVICE_TIME = BLE_UUID16_INIT(DEVICE_TIME_CHR);

/* characteristic values */
__unused static struct ble_svc_cts_local_time_info local_time_info_val = { .timezone = 4 * 3, .dst_offset = TIME_STANDARD };
static struct ble_svc_cts_curr_time current_local_time_val; static_assert(sizeof(current_local_time_val) == 10);
static struct ble_svc_cts_reference_time_info ref_time_info_val;

/* Characteristic attributes handles */
static uint16_t h_curr_time;
__unused static uint16_t h_time_zone_info;
static uint16_t h_ref_time;
static uint16_t h_device_time;

static void set_cts_unix(time_t now);
static time_t get_cts_unix() { return time(NULL); }
static void fetch_current_time(struct ble_svc_cts_curr_time *ctime);
__unused static void set_current_time(struct ble_svc_cts_curr_time* ctime);
__unused static int fetch_local_time_info(struct ble_svc_cts_local_time_info *info) { return 0; }
__unused static int set_local_time_info(struct ble_svc_cts_local_time_info info) { return 0; }
static void fetch_reference_time_info(struct ble_svc_cts_reference_time_info *info);
/* Access function */
static int curr_time_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
__unused static int time_zone_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
static int reference_time_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
static int device_time_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);

__NOINIT_ATTR static time_t last_updated;
__NOINIT_ATTR static time_t last_updated_crc;
__NOINIT_ATTR static uint8_t adjust_reason;

static const struct ble_gatt_svc_def ble_svc_cts_defs[] = { 
    {	/* Current Time Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_CTS.u,
        .characteristics = (struct ble_gatt_chr_def[]) { 
        {	/* Current Time characteristic */
            .uuid = &CHR_CURRENT_TIME.u,
            .access_cb = curr_time_chr_access,
            .flags = BLE_GATT_CHR_F_READ //| BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN | BLE_GATT_CHR_F_NOTIFY
			, .val_handle = &h_curr_time,
	    }, 
#ifdef TIME_ZONE_CHR
		{	/* Local info characteristic */
            .uuid = &CHR_TIME_ZONE.u,
            .access_cb = time_zone_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,//BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_AUTHOR,
            .val_handle = &h_time_zone_info,
            
	    }, 
#endif
		{	/* Reference time info Characteristic */
            .uuid = &CHR_REF_TIME_INFO.u,
            .access_cb = reference_time_chr_access,
            .flags = BLE_GATT_CHR_F_READ,
            .val_handle = &h_ref_time,
        },  
		{
            .uuid = &CHR_DEVICE_TIME.u,
            .access_cb = device_time_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,//BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_ENC,
            .val_handle = &h_device_time,
        }, { /* No more characteristics in this service. */ } 
        },
    }, { /* No more services. */ },
};

static void set_noinit_values(time_t val) {
	last_updated = val;
	last_updated_crc = last_updated + adjust_reason + 1;
}

void gatt_cts_service_init() {
    CHECK_(ble_gatts_count_cfg(ble_svc_cts_defs));
    CHECK_(ble_gatts_add_svcs(ble_svc_cts_defs));
	setenv("TZ", "MSK-3", 1);
	tzset();
	if(last_updated != (last_updated_crc - adjust_reason - 1)) 
	{last_updated_crc = last_updated = 0L; adjust_reason = 0;}
	ESP_LOGI(TAG, "last_updated %lu", last_updated);
}

void set_cts_unix(time_t now) {
    settimeofday((struct timeval *)&(struct timeval64) {.tv_sec = now }, NULL);
	adjust_reason = MANUAL_TIME_UPDATE_MASK;
    set_noinit_values(now);//gettimeofday(&last_updated,/*  &tmz */NULL);  /* set the last updated */
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
    }; struct timeval64 tv_now = {.tv_sec = mktime(&timeinfo) };
    settimeofday((struct timeval *)&(struct timeval64) {.tv_sec = mktime(&timeinfo) }, NULL);
	adjust_reason = ctime->adjust_reason;
	set_noinit_values(tv_now.tv_sec);  //gettimeofday(&last_updated, NULL);
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
    //gettimeofday(&tv_now, NULL); 
	time_t tv_sec = time(NULL); /* ignore microseconds */
	tv_sec -= last_updated; /* subtract the time when the last time was updated */
    info->time_source = TIME_SOURCE_MANUAL;//4
    info->time_accuracy = 0;
    uint32_t days_since_update = (tv_sec / 86400UL);
    if(days_since_update <= 255) {
		info->days_since_update = days_since_update;
        uint32_t hours_since_update = (tv_sec - days_since_update) / 3600;//(tv_sec % 86400UL) / 3600;
        info->hours_since_update = hours_since_update;
    } else { info->hours_since_update = info->days_since_update = 255; }
    //adjust_reason = (CHANGE_OF_DST_MASK | CHANGE_OF_TIME_ZONE_MASK);  
	ESP_LOGI(TAG, "%s last_updated %lu", __FUNCTION__, last_updated);
}

int curr_time_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    //ESP_LOGD(TAG, "%s  conn_handle %u attr_handle %u",ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR ? "WRITE_CHR" :  
    //ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR ? "READ_CHR"  : ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC ? "WRITE_DSC" :
    //ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC ? "READ_DSC"  : "op?", conn_handle, attr_handle);
	if (attr_handle != h_curr_time) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
    switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            fetch_current_time(&current_local_time_val);
            CHECK_RET(os_mbuf_append(ctxt->om, &current_local_time_val, sizeof(current_local_time_val)));
		}
		return 0;
#ifdef CURRENT_TIME_WRITE
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            struct ble_svc_cts_curr_time curr_time = {};
            CHECK_RET(ble_hs_mbuf_to_flat(ctxt->om, &curr_time, sizeof(curr_time), NULL));
            CHECK_RET(ble_svc_cts_curr_time_validate(curr_time));
            set_current_time(&curr_time); 
			current_local_time_val = curr_time;
            //ble_gatts_chr_updated(attr_handle); /* schedule notifications for subscribed peers */
        } return 0;
		
#endif	
		default: return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
	}
    return BLE_ATT_ERR_UNLIKELY;
}

int time_zone_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (attr_handle != h_time_zone_info) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
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
			//ble_gatts_chr_updated(h_time_zone_info);
        } return 0;
		default: return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }
	return BLE_ATT_ERR_UNLIKELY;
}

int reference_time_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (attr_handle != h_ref_time) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        fetch_reference_time_info(&ref_time_info_val);
        CHECK_RET(os_mbuf_append(ctxt->om, &ref_time_info_val, sizeof(ref_time_info_val))); 
        return 0;
    } else if(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
	//ESP_LOGW(TAG, "REF_TIME_INFO ctxt->op %u", ctxt->op);
	return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

int device_time_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
	if (attr_handle != h_device_time) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            CHECK_RET(os_mbuf_append(ctxt->om, &(time_t) { get_cts_unix() }, sizeof(time_t)));
        } return 0;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            if(ctxt->om->om_len != sizeof(time_t)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            set_cts_unix(*(time_t*)ctxt->om->om_data);
		} return 0;
		default: return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }
	return BLE_ATT_ERR_UNLIKELY;
}


