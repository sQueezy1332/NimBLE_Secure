/*
* Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
*
* Licensed to the Apache Software Foundation (ASF) under one
* or more contributor license agreements.  See the NOTICE file
* distributed with this work for additional information
* regarding copyright ownership.  The ASF licenses this file
* to you under the Apache License, Version 2.0 (the
* "License"); you may not use this file except in compliance
* with the License.  You may obtain a copy of the License at
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
* KIND, either express or implied.  See the License for the
* specific language governing permissions and limitations
* under the License.
*/


#include "syscfg/syscfg.h"
#include "host/ble_hs_log.h"

#include <string.h>
#include <esp_system.h>
#include "sysinit/sysinit.h"
#include "host/ble_hs.h"
#include "ble_store_config.h"
#include "ble_store_config_priv.h"
#include "esp_log.h"
#include "nvs.h"
#include "../../../src/ble_hs_resolv_priv.h"

#if MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)

#define NIMBLE_NVS_STR_NAME_MAX_LEN		16
#define NIMBLE_NVS_PEER_SEC_KEY 		"peer_sec"
#define NIMBLE_NVS_OUR_SEC_KEY			"our_sec"
#define NIMBLE_NVS_CCCD_SEC_KEY			"cccd_sec"
#define NIMBLE_NVS_CSFC_SEC_KEY 		"csfc_sec"
#define NIMBLE_NVS_PEER_RECORDS_KEY		"p_dev_rec"
#define NIMBLE_NVS_EAD_SEC_KEY 			"ead_sec"
#define NIMBLE_NVS_LOCAL_IRK_KEY		"local_irk"
#define NIMBLE_NVS_RPA_RECORDS_KEY		"rpa_rec"
#define NIMBLE_NVS_NAMESPACE			"nimble_bond"

static const char* TAG = "NIMBLE_NVS";

/*****************************************************************************
 * $ MISC                                                                    *
 *****************************************************************************/

static inline __always_inline bool compare_str(const char* buf, const char* key) {
	return *(uint32_t*)buf == *(uint32_t*)key;
}

__unused static void get_nvs_key_string(int obj_type, int index, char* key_string) {

	if (likely(obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC)) {
		sprintf(key_string, "%s_%d", NIMBLE_NVS_PEER_SEC_KEY, index);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)			
	} else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
		sprintf(key_string, "%s_%d", NIMBLE_NVS_PEER_RECORDS_KEY, index);
#endif
		/*} else if (obj_type == BLE_STORE_OBJ_TYPE_OUR_SEC) {
			sprintf(key_string, "%s_%d", NIMBLE_NVS_OUR_SEC_KEY, index);*/
#if MYNEWT_VAL(ENC_ADV_DATA)
	} else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
		sprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_EAD_SEC_KEY, index);
#endif
	} else if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
		sprintf(key_string, "%s_%d", NIMBLE_NVS_LOCAL_IRK_KEY, index);
	/* else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR) {
		sprintf(key_string, "%s_%d", NIMBLE_NVS_RPA_RECORDS_KEY, index); */
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)		
	} else if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
		sprintf(key_string, "%s_%d", NIMBLE_NVS_CCCD_SEC_KEY, index);
#endif
	}// else { sprintf(key_string, "%s_%d", NIMBLE_NVS_CSFC_SEC_KEY, index);  }
	else { *key_string = '\0'; }
}

static void set_nvs_key_index(char* end_str, size_t index) {
	sprintf(end_str, "_%u", index);
}

/* compares values at two addresses of size = item_size
* @Returns               index if entries match
*                       -1 if mismatch
*/
static int 
get_nvs_matching_index(const void* nvs_val, const void* restrict db_list, size_t db_num, size_t item_size, size_t field_size) {
	for (size_t i = 0; i < db_num; i++) {
		if (memcmp(nvs_val, db_list, field_size) == 0) {
			return i; // Key matches with the one in RAM database
		}
		db_list += item_size;
	}
	return -1;
}

/**  Get the maximum number of entries allowed for this object type */
__unused static size_t get_nvs_max_obj_value(int obj_type) {
	if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
		return 1;
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS) 
	} else if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
		return MYNEWT_VAL(BLE_STORE_MAX_CCCDS);
#endif
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
	} else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
		return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
#endif
	//} else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) { 
		return MYNEWT_VAL(BLE_STORE_MAX_CSFCS);
#if MYNEWT_VAL(ENC_ADV_DATA)
	} else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
		return MYNEWT_VAL(BLE_STORE_MAX_EADS);
#endif
	} return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
}

static size_t get_nvs_max_obj_value_by_key(const char* key) {
	if (compare_str(key, NIMBLE_NVS_LOCAL_IRK_KEY)) {
		return 1;
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS) 
	} else if (compare_str(key, NIMBLE_NVS_CCCD_SEC_KEY)) {
		return MYNEWT_VAL(BLE_STORE_MAX_CCCDS);
#endif
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
	//} else if(compare_str(key, NIMBLE_NVS_CSFC_SEC_KEY)) { 
		return MYNEWT_VAL(BLE_STORE_MAX_CSFCS);
#endif
#if MYNEWT_VAL(ENC_ADV_DATA)
	} else if (compare_str(key, NIMBLE_NVS_EAD_SEC_KEY)) {
		return MYNEWT_VAL(BLE_STORE_MAX_EADS);
#endif
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
	} else if (compare_str(key, NIMBLE_NVS_PEER_RECORDS_KEY)) {
		return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
#endif
	} return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
}

/*****************************************************************************
 * $ NVS                                                                     *
 *****************************************************************************/

__unused static size_t get_expected_size_for_obj_type(int obj_type)
{
	switch (obj_type) {
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS) 
	case BLE_STORE_OBJ_TYPE_CCCD: return sizeof(struct ble_store_value_cccd);
#endif
	//case BLE_STORE_OBJ_TYPE_CSFC: return sizeof(struct ble_store_value_csfc);
#if MYNEWT_VAL(ENC_ADV_DATA)
	case BLE_STORE_OBJ_TYPE_ENC_ADV_DATA: return sizeof(struct ble_store_value_ead);
#endif
	case BLE_STORE_OBJ_TYPE_LOCAL_IRK: return sizeof(struct ble_store_value_local_irk);
	//case BLE_STORE_OBJ_TYPE_PEER_ADDR: return sizeof(struct ble_store_value_rpa_rec);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
	case BLE_STORE_OBJ_TYPE_PEER_DEV_REC: return sizeof(struct ble_hs_dev_records);
#endif
	case BLE_STORE_OBJ_TYPE_PEER_SEC: 
	case BLE_STORE_OBJ_TYPE_OUR_SEC: return sizeof(struct ble_store_nvs);
	default: return 0;
	}
}

static size_t get_expected_size_for_obj_by_key(const char* key)
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	if (compare_str(key, NIMBLE_NVS_PEER_SEC_KEY)) {
		return sizeof(struct ble_store_nvs);
	//} else if (compare_str(key, NIMBLE_NVS_OUR_SEC_KEY)) { 
		return sizeof(struct ble_store_nvs);
#endif
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS) 
	} else if (compare_str(key, NIMBLE_NVS_CCCD_SEC_KEY)) {
		return sizeof(struct ble_store_value_cccd);
#endif
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
	//} else if(compare_str(key, NIMBLE_NVS_CSFC_SEC_KEY)) { 
		return sizeof(struct ble_store_value_csfc);
#endif
#if MYNEWT_VAL(ENC_ADV_DATA)
	} else if (compare_str(key, NIMBLE_NVS_EAD_SEC_KEY)) {
		return sizeof(struct ble_store_value_ead);
#endif
	} else if (compare_str(key, NIMBLE_NVS_LOCAL_IRK_KEY)) {
		return sizeof(struct ble_store_value_local_irk);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
	} else if (compare_str(key, NIMBLE_NVS_PEER_RECORDS_KEY)) {
		return sizeof(struct ble_hs_dev_records);
#endif
	}
	return 0;
}

static int get_nvs_db_value(const char* key_string, void* val)
{
	nvs_handle_t nimble_handle;
	esp_err_t ret = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READONLY, &nimble_handle);
	if (ret != ESP_OK) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
		return BLE_HS_ESTORE_FAIL;
	}
	size_t required_size;
	ret = nvs_get_blob(nimble_handle, key_string, NULL, &required_size);
	/* if Address pointer for value is NULL, filling of value not needed */
	if (val && ret == ESP_OK) {
		size_t expected_size = get_expected_size_for_obj_by_key(key_string); // Validate that NVS data size matches expected struct size
		if (likely(required_size == expected_size)) {
			ret = nvs_get_blob(nimble_handle, key_string, val, &required_size);
		} else {
			ESP_LOGE(TAG, "data size mismatch for %s: expected %u, got %u", key_string, expected_size, required_size);
			ret = ESP_ERR_NVS_INVALID_LENGTH;
		}
	}
	nvs_close(nimble_handle);
	return ret;
}

/* Finds empty index or total count or index to be deleted in NVS database
* This function serves 3 different purposes depending upon 'empty' and `value`
* arguments.
* @ returns             - empty NVS index, if empty = true
*                       - count of NVS database, if empty = 0, value = NULL
*                       - index that does not match with RAM db, if empty = 0 & value has database address.
*/
static int get_nvs_db_attribute(char pkey[], bool empty, void* restrict value, size_t num_value)
{
	uint8_t buf[96]; uint8_t* pbuf;
	char* end_str = pkey + strlen(pkey);
	int ret; size_t count = 0, obj_size, field_size;
	if(value) { 
		obj_size = get_expected_size_for_obj_by_key(pkey); 
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
		if (compare_str(pkey, NIMBLE_NVS_PEER_RECORDS_KEY)){
			pbuf = &((struct ble_hs_dev_records*)buf)->peer_sec;
			value = &((struct ble_hs_dev_records*)value)->peer_sec;
			field_size = sizeof(struct ble_hs_peer_sec);
		} else
#endif
#if MYNEWT_VAL(ENC_ADV_DATA)
		if (compare_str(pkey, NIMBLE_NVS_EAD_SEC_KEY)) {
			pbuf = &((struct ble_hs_dev_records*)buf)->peer_sec;
			value = &((struct ble_store_value_ead*)value)->peer_addr;
			field_size = sizeof(ble_addr_t);
		} else
#endif
		{
			pbuf = buf;
			field_size = obj_size;
		}
	} else { pbuf = NULL; }
	size_t max_limit = get_nvs_max_obj_value_by_key(pkey);
	for (size_t i = 1; i <= max_limit; i++) {
		set_nvs_key_index(end_str, i);
		ret = get_nvs_db_value(pkey, pbuf);
		// Check if the user is searching for empty index to write to
		if (ret == ESP_ERR_NVS_NOT_FOUND) {
			if (empty) {
				ESP_LOGD(TAG, "Empty index found: %s", pkey);
				return i;
			}
		} else if (ret == ESP_OK) {
			count++;
			// If user has provided value, then the purpose is to find non-matching entry from NVS
			if (value) {
				ret = get_nvs_matching_index(pbuf, value, num_value, obj_size, field_size);
				//If found non-matching/odd entry of NVS with entries in theinternal database, return NVS index so can be deleted 
				if (ret == -1 && !empty) {
					return i;
				}
			}
		} else {
			ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
			//ESP_LOGE(TAG, "read operation failed while fetching size");
			return -1;
		}
	}
	if (!empty) {
		if (!value) {
			*end_str = '\0';
			return count;
		}
		ESP_LOGW(TAG, "No non-matching entry found %s", pkey);
		return -1;
	}
	return (max_limit + 1);
}

/* Deletes NVS value at given index
* @Returns               0 on success,
*                       -1 on NVS memory access failure
*/
static int ble_nvs_delete_value(const char* key)
{
	/*if (index > get_nvs_max_obj_value(obj_type)) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EUNKNOWN);
		return BLE_HS_EUNKNOWN;
	}*/ //index form attribute can not be bigger than max_obj_value
	nvs_handle_t nimble_handle;
	esp_err_t ret = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
	if (ret != ESP_OK) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
		return BLE_HS_ESTORE_FAIL;
	}
	ret = nvs_erase_key(nimble_handle, key); // Erase the key with given index 
	if (ret == ESP_OK) {
		ret = nvs_commit(nimble_handle);
	}
	nvs_close(nimble_handle);
	if (unlikely(ret)) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
		return BLE_HS_ESTORE_FAIL;
	}
	return 0;
}

static int ble_nvs_write_key_value(const char* key, const void* value, size_t required_size)
{
	nvs_handle_t nimble_handle;
	esp_err_t ret = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = nvs_set_blob(nimble_handle, key, value, required_size);
	if (ret == ESP_OK) {
		ret = nvs_commit(nimble_handle);
	}
	nvs_close(nimble_handle);
	return ret;
}

/* To write key value in NVS.
* @Returns              0 if success
*                       BLE_HS_ESTORE_FAIL if failure
*                       BLE_HS_ESTORE_CAP if no space in NVS
*/
static int ble_store_nvs_write(char key[], const void* val)
{
	int empty_index = get_nvs_db_attribute(key, true, NULL, 0);
	if (unlikely(empty_index == -1)) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	} else if (unlikely(empty_index > get_nvs_max_obj_value_by_key(key))) {
		//bare-bone config code will take care of capacity overflow event, however another check added for consistency
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
		return BLE_HS_ESTORE_CAP;
	}
	int ret = ble_nvs_write_key_value(key, val, get_expected_size_for_obj_by_key(key));
	if (unlikely(ret)) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
		return BLE_HS_ESTORE_FAIL;
	}
	return 0;
}

static int populate_db_from_nvs(const char* key, void* restrict db_item, size_t* db_num)
{
	uint8_t buf[96]; esp_err_t ret; static_assert(sizeof(union ble_store_value) < 96);
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN]; 
	char* end_str = key_string + sprintf(key_string, key);
	size_t max_entries = get_nvs_max_obj_value_by_key(key);
	size_t item_size = get_expected_size_for_obj_by_key(key);
	for (size_t i = 1; i <= max_entries; i++) {
		if (*db_num >= max_entries) {
			ESP_LOGW(TAG, "%s RAM array full, break", key_string);
			break;
		}
		set_nvs_key_index(end_str, i);
		ret = get_nvs_db_value(key_string, buf);
		if (ret == ESP_ERR_NVS_NOT_FOUND) {
			continue;
		} else if (ret != ESP_OK) {
			ESP_LOGE(TAG, "populate failed for %s", key);
			ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
			return -1;
		} else { ESP_LOGD(TAG, "%s filled up", key_string); }
		memcpy(db_item, buf, item_size);
		db_item += item_size;
		++(*db_num);
	}
	return 0;
}

extern int ble_store_compare_bond_count(const void* a, const void* b);
/* Gets the database in RAM filled up with keys stored in NVS. The sequence of the keys in database may get lost.
 */
static int ble_nvs_restore_sec_keys(void)
{
	int ret;
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	/*ret = populate_db_from_nvs(NIMBLE_NVS_OUR_SEC_KEY, ble_store_config_our_secs, &ble_store_config_num_our_secs);
	if (ret != ESP_OK) { return ret; }
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_OUR_SEC_KEY, ble_store_config_num_our_secs);*/
	ret = populate_db_from_nvs(NIMBLE_NVS_PEER_SEC_KEY, ble_store_peer_secs, &ble_store_num_peer_secs);
	if (ret != ESP_OK) { return ret; }
	/* Only check for out-of-order entries if we have valid data */
	/*if (//ble_store_config_num_our_secs > 1 || 
		 ble_store_num_peer_secs > 1) {
		int our_check_limit = (ble_store_config_num_our_secs > 1) ? (ble_store_config_num_our_secs - 1) : 0;
		size_t peer_check_limit = (ble_store_num_peer_secs > 1) ? (ble_store_num_peer_secs - 1) : 0;
		size_t check_limit = (our_check_limit > peer_check_limit) ? our_check_limit :
			 peer_check_limit;
		for (size_t i = 0; i < check_limit; i++) {
			if (//(i < our_check_limit &&
				//ble_store_config_our_secs[i].bond_count > ble_store_config_our_secs[i+1].bond_count) ||
				(i < peer_check_limit && 
				 ble_store_peer_secs[i].bond_count > ble_store_peer_secs[i + 1].bond_count)
				) { ret = 1;
				break;
			}
		}
	}*/
	if (ret) {
		//qsort(ble_store_config_our_secs, ble_store_config_num_our_secs, sizeof(struct ble_store_nvs), ble_store_compare_bond_count);
		//qsort(ble_store_peer_secs, ble_store_num_peer_secs, sizeof(struct ble_store_nvs), ble_store_compare_bond_count);
	}
	// Only access array if we have valid entries to prevent index -1 access
	//if (ble_store_config_num_our_secs > 0) { ble_store_config_our_bond_count = ble_store_config_our_secs[ble_store_config_num_our_secs - 1].bond_count;
	//} else { ble_store_config_our_bond_count = 0; }
	//if (ble_store_num_peer_secs > 0) { //ble_store_peer_bond_count = ble_store_peer_secs[ble_store_num_peer_secs - 1].bond_count; 
	//} else { ble_store_peer_bond_count = 0;}
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_PEER_SEC_KEY, ble_store_num_peer_secs);
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
	ret = populate_db_from_nvs(NIMBLE_NVS_CCCD_SEC_KEY, ble_store_config_cccds, &ble_store_config_num_cccds);
	if (ret) { return ret; }
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_CCCD_SEC_KEY, ble_store_config_num_cccds);
#endif
/*
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
	ret = populate_db_from_nvs(NIMBLE_NVS_CSFC_SEC_KEY, ble_store_config_csfcs, &ble_store_config_num_csfcs);
	if (ret) { return ret; }
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_CSFC_SEC_KEY, ble_store_config_num_csfcs);
#endif
*/
#if MYNEWT_VAL(ENC_ADV_DATA)
	ret = populate_db_from_nvs(NIMBLE_NVS_EAD_SEC_KEY, ble_store_config_eads, &ble_store_config_num_eads);
	if (ret) { return ret; }
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_EAD_SEC_KEY, ble_store_config_num_eads);
#endif
	size_t ble_store_num_local_irks = 0;
	ret = populate_db_from_nvs(NIMBLE_NVS_LOCAL_IRK_KEY, ble_store_local_irk, &ble_store_num_local_irks);
	if (ret) { return ret; }
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_LOCAL_IRK_KEY, ble_store_num_local_irks);
/*
	err = populate_db_from_nvs(NIMBLE_NVS_RPA_RECORDS_KEY, ble_store_config_rpa_recs, &ble_store_config_num_rpa_recs);
	if (err != ESP_OK) { return err; } 
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_RPA_RECORDS_KEY, ble_store_config_num_rpa_recs);
*/
	return 0;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int ble_nvs_restore_peer_records(void)
{
	int ble_store_num_peer_dev_rec = 0;
	struct ble_hs_dev_records* peer_dev_rec = ble_rpa_get_peer_dev_records();
	if (peer_dev_rec == NULL) return BLE_HS_ENOMEM;
	esp_err_t ret = populate_db_from_nvs(NIMBLE_NVS_PEER_RECORDS_KEY, peer_dev_rec, &ble_store_num_peer_dev_rec);
	if (ret != ESP_OK) { return ret; }
	ble_rpa_set_num_peer_dev_records(ble_store_num_peer_dev_rec);
	ESP_LOGI(TAG, "%s restored: %u", NIMBLE_NVS_PEER_RECORDS_KEY, ble_store_num_peer_dev_rec);
	return 0;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
int ble_store_persist_cccds(void)
{
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_CCCD_SEC_KEY;
	int nvs_count = get_nvs_db_attribute(&key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_cccds) {
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_config_cccds[ble_store_config_num_cccds - 1]);
	} else if (nvs_count > ble_store_config_num_cccds) {
		nvs_count = get_nvs_db_attribute(key_string, 0, ble_store_config_cccds, ble_store_config_num_cccds);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
int ble_store_persist_csfcs(void)
{
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_CSFC_SEC_KEY;
	int nvs_count = get_nvs_db_attribute(key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_csfcs) {
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_config_csfcs[ble_store_config_num_csfcs - 1]);
	} else if (nvs_count > ble_store_config_num_csfcs) {
		nvs_count = get_nvs_db_attribute(key_string, 0, ble_store_config_csfcs, ble_store_config_num_csfcs);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}
#endif

#if MYNEWT_VAL(ENC_ADV_DATA)
int ble_store_persist_eads(void)
{
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_EAD_SEC_KEY;
	int nvs_count = get_nvs_db_attribute(&key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_eads) {
		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_config_eads[ble_store_config_num_eads - 1]);
	} else if (nvs_count > ble_store_config_num_eads) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(key_string, 0, ble_store_config_eads, ble_store_config_num_eads);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}
#endif

int ble_store_persist_local_irk(void)
{
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_LOCAL_IRK_KEY;
	int nvs_count = get_nvs_db_attribute(key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	int has = *(uint64_t*)&ble_store_local_irk[0];
	if (nvs_count < 1 && has) {
		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_local_irk[0]);
	} else if (nvs_count > 0 && !has) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(key_string, 0, ble_store_local_irk, 1);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}

int ble_store_persist_rpa_recs(void)
{
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_RPA_RECORDS_KEY;
	int nvs_count = get_nvs_db_attribute(key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_rpa_recs) {
		// NVS db count less than RAM count, write operation
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_config_rpa_recs[ble_store_config_num_rpa_recs - 1]);
	} else if (nvs_count > ble_store_config_num_rpa_recs) {
		//NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(key_string, 0,  ble_store_config_rpa_recs, ble_store_config_num_rpa_recs);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for RPA_REC");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}

int ble_store_persist_peer_secs(void) //1220785 //after 1220545
{
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_PEER_SEC_KEY;
	int nvs_count = get_nvs_db_attribute(key_string, 0, NULL, 0);
	if (nvs_count < 0) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_num_peer_secs) {
		// NVS db count less than RAM count, write operation 
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_peer_secs[ble_store_num_peer_secs - 1]);
	} else if (nvs_count > ble_store_num_peer_secs) {
		// NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(key_string, 0, ble_store_peer_secs, ble_store_num_peer_secs);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}

int ble_store_persist_our_secs(void)
{/*
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_OUR_SEC_KEY;
	int nvs_count = get_nvs_db_attribute(key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_our_secs) {
		// NVS db count less than RAM count, write operation
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &ble_store_config_our_secs[ble_store_config_num_our_secs - 1]);
	} else if (nvs_count > ble_store_config_num_our_secs) {
		//NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(key_string, 0, ble_store_config_our_secs, ble_store_config_num_our_secs);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}*/
	return 0;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
int ble_store_persist_peer_records(void)
{
	struct ble_hs_dev_records* peer_dev_rec = ble_rpa_get_peer_dev_records();
	if (peer_dev_rec == NULL) {
		return BLE_HS_ENOMEM;
	}
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN] = NIMBLE_NVS_PEER_RECORDS_KEY;
	int ble_store_num_peer_dev_rec = ble_rpa_get_num_peer_dev_records();
	int nvs_count = get_nvs_db_attribute(key_string, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_num_peer_dev_rec) {
		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting %s", key_string);
		return ble_store_nvs_write(key_string, &peer_dev_rec[ble_store_num_peer_dev_rec - 1]);
	} else if (nvs_count > ble_store_num_peer_dev_rec) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(key_string, 0, peer_dev_rec, ble_store_num_peer_dev_rec);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting %s", key_string);
		return ble_nvs_delete_value(key_string);
	}
	return 0;
}
#endif

void ble_store_nvs_init(void)
{
	int err = ble_nvs_restore_sec_keys();
	if (err != 0) {
		ESP_LOGE(TAG, "NVS operation failed, can't retrieve the bonding info");
	}
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
	err = ble_nvs_restore_peer_records();
	//if (err != 0) { ESP_LOGE(TAG, "NVS operation failed, can't retrieve the peer records"); }
#endif
}
#endif //#if MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)
