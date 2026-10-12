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
#define NIMBLE_NVS_NAMESPACE			'nimble_bond'

static const char* TAG = "NIMBLE_NVS";

/*****************************************************************************
 * $ MISC                                                                    *
 *****************************************************************************/

static void get_nvs_key_string(int obj_type, int index, char* key_string) {

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

/* compares values at two addresses of size = item_size
* @Returns               index if entries match
*                       -1 if mismatch
*/
static int get_nvs_matching_index(const void* nvs_val, const void* restrict db_list, size_t db_num, size_t item_size) {
	for (size_t i = 0; i < db_num; i++) {
		if (memcmp(nvs_val, db_list, item_size) == 0) {
			return i; // Key matches with the one in RAM database
		}
		db_list += item_size;
	}
	return -1;
}

#if MYNEWT_VAL(ENC_ADV_DATA)
/* EAD entries contain a pointer (km), so raw memcmp() is not valid.
 * Match by stable key (peer_addr) to determine if an NVS entry exists in RAM.
 */
static int
get_nvs_matching_ead_index(const struct ble_store_value_ead* nvs_val,
	void* db_list, int db_num)
{
	struct ble_store_value_ead* db_item = (struct ble_store_value_ead*)db_list;
	int i;

	for (i = 0; i < db_num; i++) {
		if (ble_addr_cmp(&nvs_val->peer_addr, &db_item->peer_addr) == 0) {
			return i;
		}
		db_item++;
	}

	return -1;
}
#endif
/**  Get the maximum number of entries allowed for this object type */
static size_t get_nvs_max_obj_value(int obj_type) {
	/* If host based privacy is enabled */
	if (unlikely(obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK)) {
		return 1;
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS) 
	} else if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
		return MYNEWT_VAL(BLE_STORE_MAX_CCCDS);
#endif
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
	} else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
		return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
#endif
		//} else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) { return MYNEWT_VAL(BLE_STORE_MAX_CSFCS);
#if MYNEWT_VAL(ENC_ADV_DATA)
	} else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
		return MYNEWT_VAL(BLE_STORE_MAX_EADS);
#endif
	} return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
}

/*****************************************************************************
 * $ NVS                                                                     *
 *****************************************************************************/

static size_t get_expected_size_for_obj_type(int obj_type)
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

static int get_nvs_db_value(int obj_type, const char* key_string, void* val)
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
		size_t expected_size = get_expected_size_for_obj_type(obj_type); // Validate that NVS data size matches expected struct size
		if (likely(required_size == expected_size)) {
			ret = nvs_get_blob(nimble_handle, key_string, val, &required_size);
		} else {
			ESP_LOGE(TAG, "data size mismatch for obj_type %d: expected %u, got %u", obj_type, expected_size, required_size);
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
static int get_nvs_db_attribute(int obj_type, bool empty, void* restrict value, size_t num_value)
{
	uint8_t buf[96]; uint8_t * const pbuf = value ? buf : NULL;
	int ret; size_t count = 0;
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
	size_t max_limit = get_nvs_max_obj_value(obj_type);
	for (size_t i = 1; i <= max_limit; i++) {
		get_nvs_key_string(obj_type, i, key_string);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
		if (obj_type != BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
#endif
			ret = get_nvs_db_value(obj_type, key_string, pbuf);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
		} else {
			ret = get_nvs_peer_record(key_string, pbuf);
		}
#endif
		// Check if the user is searching for empty index to write to
		if (ret == ESP_ERR_NVS_NOT_FOUND) {
			if (empty) {
				ESP_LOGD(TAG, "Empty index found = %u for obj_type = %d", i, obj_type);
				return i;
			}
		} else if (ret == ESP_OK) {
			count++;
			// If user has provided value, then the purpose is to find non-matching entry from NVS
			if (value) {
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
				if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
					struct ble_hs_dev_records* recs = value;
					ret = -1;
					for (size_t j = 0; j < num_value; j++) {
						if (memcmp(&buf, &recs[j].peer_sec, sizeof(struct ble_hs_peer_sec)) == 0) {
							ret = j;
							break;
						}
					}
				} else
#endif
				{
					ret = get_nvs_matching_index(&buf, value, num_value, get_expected_size_for_obj_type(obj_type));
				}
				//If found non-matching/odd entry of NVS with entries in theinternal database, return NVS index so can be deleted 
				if (ret == -1 && !empty) {
					return i;
				}
			}
		} else {
			ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
			ESP_LOGE(TAG, "read operation failed while fetching size");
			return -1;
		}
	}
	if (!empty) {
		if (!value) {
			return count;
		}
		return -1; // No non-matching entry found
	}
	return (max_limit + 1);
}

/* Deletes NVS value at given index
* @Returns               0 on success,
*                       -1 on NVS memory access failure
*/
static int ble_nvs_delete_value(int obj_type, size_t index)
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
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
	get_nvs_key_string(obj_type, index, key_string);
	ret = nvs_erase_key(nimble_handle, key_string); // Erase the key with given index 
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
static int ble_store_nvs_write(int obj_type, const void* val)
{
	int empty_index = get_nvs_db_attribute(obj_type, true, NULL, 0);
	if (unlikely(empty_index == -1)) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	} else if (unlikely(empty_index > get_nvs_max_obj_value(obj_type))) {
		//bare-bone config code will take care of capacity overflow event, however another check added for consistency
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
		return BLE_HS_ESTORE_CAP;
	}
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
	get_nvs_key_string(obj_type, empty_index, key_string);
	int ret = ble_nvs_write_key_value(key_string, val, get_expected_size_for_obj_type(obj_type));
	if (unlikely(ret)) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
		return BLE_HS_ESTORE_FAIL;
	}
	return 0;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int ble_store_nvs_peer_records(int obj_type, const struct ble_hs_dev_records* p_dev_rec)
{
	int empty_index = get_nvs_db_attribute(obj_type, true, NULL, 0);
	if (empty_index == -1) {
		ESP_LOGE(TAG, "NVS operation failed !!");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	} else if (empty_index > get_nvs_max_obj_value(obj_type)) {
		// bare-bone config code will take care of capacity overflow event, however another check added for consistency
		ESP_LOGD(TAG, "NVS size overflow.");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
		return BLE_HS_ESTORE_CAP;
	}
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
	get_nvs_key_string(obj_type, empty_index, key_string);
	return ble_nvs_write_key_value(key_string, p_dev_rec, sizeof(struct ble_hs_dev_records));
}
#endif

static int populate_db_from_nvs(int obj_type, void* restrict db_item, size_t* restrict db_num)
{
	uint8_t buf[96]; esp_err_t ret; static_assert(sizeof(union ble_store_value) < 96);
	char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
	size_t max_entries = get_nvs_max_obj_value(obj_type);
	for (size_t i = 1, item_size = 0; i <= max_entries; i++) {
		get_nvs_key_string(obj_type, i, key_string);
		ret = get_nvs_db_value(obj_type, key_string, &buf);
		if (ret == ESP_ERR_NVS_NOT_FOUND) {
			continue;
		} else if (ret != ESP_OK) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
			return -1;
		}
		if (unlikely(obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK)) {
			if (*db_num >= sizeof(ble_store_local_irk) / sizeof(ble_store_local_irk[0])) {
				ESP_LOGW(TAG, "Local IRK%s%d", ": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "Local IRK%s%d", " is filled up from index = ", i);
			item_size = sizeof(struct ble_store_value_local_irk);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
		} else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
			if (*db_num >= max_entries) {
				ESP_LOGW(TAG, "Peer dev records%s%d", ": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "Peer dev records%s%d", " is filled up from index = ", i);
			item_size = sizeof(struct ble_hs_dev_records);
#endif
		/*}  else if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
			if ((int)*db_num >= MYNEWT_VAL(BLE_STORE_MAX_CCCDS)) {
				ESP_LOGW(TAG, "CCCD%s %d",": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "CCCD%s%d", " is filled up from index = ", i);
			item_size = sizeof(struct ble_store_value_cccd);
		} else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
			if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_CSFCS)) {
				ESP_LOGW(TAG, "CSFC%s%d",": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "CSFC%s%d", " is filled up from index = ", i);
			item_size = sizeof(struct ble_store_value_csfc);*/
#if MYNEWT_VAL(ENC_ADV_DATA)
		} else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
			if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_EADS)) {
				ESP_LOGW(TAG, "EAD%s%d", ": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "EAD%s%d", " is filled up from index = ", i);
			item_size = sizeof(struct ble_store_value_ead);
#endif
		/* } else if(obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR) {
			if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
				ESP_LOGW(TAG, "RPA_REC%s%d",": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "RPA_REC%s%d"," is filled up from index = ", i);
			item_size = sizeof(struct ble_store_value_rpa_rec);*/
		} else {
			if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
				ESP_LOGW(TAG, "SEC%s%d", ": RAM array full, skipping index ", i);
				continue;
			}
			ESP_LOGD(TAG, "KEY%s%d", " is filled up from index = ", i);
			item_size = sizeof(struct ble_store_nvs);
		}
		memcpy(db_item, &buf, item_size);
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
	int err;
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	/*err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_OUR_SEC, ble_store_config_our_secs, &ble_store_config_num_our_secs);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'our sec'");
		return err;
	}
	ESP_LOGD(TAG, "ble_store_config_our_secs restored %d bonds", ble_store_config_num_our_secs);*/

	err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_PEER_SEC, ble_store_peer_secs, &ble_store_num_peer_secs);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'peer sec'");
		return err;
	}
	/* Only check for out-of-order entries if we have valid data */
	if (/*ble_store_config_num_our_secs > 1 || */ ble_store_num_peer_secs > 1) {
		//int our_check_limit = (ble_store_config_num_our_secs > 1) ? (ble_store_config_num_our_secs - 1) : 0;
		size_t peer_check_limit = (ble_store_num_peer_secs > 1) ? (ble_store_num_peer_secs - 1) : 0;
		size_t check_limit = /*(our_check_limit > peer_check_limit) ? our_check_limit : */ peer_check_limit;
		for (size_t i = 0; i < check_limit; i++) {
			if (//(i < our_check_limit &&
				// ble_store_config_our_secs[i].bond_count > ble_store_config_our_secs[i+1].bond_count) ||
				(/*i < peer_check_limit && */ ble_store_peer_secs[i].bond_count > ble_store_peer_secs[i + 1].bond_count)) {
				err = 1;
				break;
			}
		}
	}
	if (err) {
		//qsort(ble_store_config_our_secs, ble_store_config_num_our_secs, sizeof(struct ble_store_nvs), ble_store_compare_bond_count);
		qsort(ble_store_peer_secs, ble_store_num_peer_secs, sizeof(struct ble_store_nvs), ble_store_compare_bond_count);
	}

	// Only access array if we have valid entries to prevent index -1 access
	//if (ble_store_config_num_our_secs > 0) { ble_store_config_our_bond_count = ble_store_config_our_secs[ble_store_config_num_our_secs - 1].bond_count;
	//} else { ble_store_config_our_bond_count = 0; }
	if (ble_store_num_peer_secs > 0) {
		ble_store_peer_bond_count = ble_store_peer_secs[ble_store_num_peer_secs - 1].bond_count;
	} else {
		ble_store_peer_bond_count = 0;
	}
	ESP_LOGD(TAG, "ble_store_peer_secs restored %d bonds", ble_store_num_peer_secs);
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
	err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_CCCD, ble_store_config_cccds,
		&ble_store_config_num_cccds);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'CCCD'");
		return err;
	}
	ESP_LOGD(TAG, "ble_store_config_cccds restored %d bonds", ble_store_config_num_cccds);
#endif
/*
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
	err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_CSFC, ble_store_config_csfcs, &ble_store_config_num_csfcs);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'CSFC'");
		return err;
	}
	ESP_LOGD(TAG, "ble_store_config_csfcs restored %d bonds", ble_store_config_num_csfcs);
#endif
*/
#if MYNEWT_VAL(ENC_ADV_DATA)
	err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, ble_store_config_eads,
		&ble_store_config_num_eads);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'EAD'");
		return err;
	}
	ESP_LOGD(TAG, "ble_store_config_eads restored %d bonds",
		ble_store_config_num_eads);
#endif
	size_t ble_store_num_local_irks = 0;
	err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_LOCAL_IRK, ble_store_local_irk, &ble_store_num_local_irks);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'Local IRK'");
		return err;
	}
	ESP_LOGD(TAG, "ble_store_local_irk restored %u irks", ble_store_num_local_irks);
	/*
	err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_PEER_ADDR, ble_store_config_rpa_recs, &ble_store_config_num_rpa_recs);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed for 'RPA_REC'");
		return err;
	} else{ ESP_LOGD(TAG, "ble_store_config_rpa_recs restored %d bonds", ble_store_config_num_rpa_recs); }*/
	return 0;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int ble_nvs_restore_peer_records(void)
{
	int ble_store_num_peer_dev_rec = 0;
	struct ble_hs_dev_records* peer_dev_rec = ble_rpa_get_peer_dev_records();
	if (peer_dev_rec == NULL) return BLE_HS_ENOMEM;
	esp_err_t ret = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, peer_dev_rec, &ble_store_num_peer_dev_rec);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "NVS operation failed fetching 'Peer Dev Records'");
		return ret;
	}
	ble_rpa_set_num_peer_dev_records(ble_store_num_peer_dev_rec);
	ESP_LOGD(TAG, "peer_dev_rec restored %d records", ble_store_num_peer_dev_rec);
	return 0;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
int ble_store_persist_cccds(void)
{
	union ble_store_value val;
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_CCCD, 0, NULL, 0); 
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "NVS operation failed while persisting CCCD");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}

	if (nvs_count < ble_store_config_num_cccds) {
		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting CCCD value in NVS...");
		val.cccd = ble_store_config_cccds[ble_store_config_num_cccds - 1];
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_CCCD, &val);
	} else if (nvs_count > ble_store_config_num_cccds) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_CCCD, 0, ble_store_config_cccds, ble_store_config_num_cccds);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for CCCD");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting CCCD, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_CCCD, nvs_count);
	}
	return 0;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
int ble_store_persist_csfcs(void)
{
	union ble_store_value val;
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_CSFC, 0, NULL, 0);
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "NVS operation failed while persisting CSFC");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_csfcs) {
		// NVS db count less than RAM count, write operation
		ESP_LOGD(TAG, "Persisting CSFC value in NVS...");
		val.csfc = ble_store_config_csfcs[ble_store_config_num_csfcs - 1];
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_CSFC, &val);
	} else if (nvs_count > ble_store_config_num_csfcs) {
		// NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_CSFC, 0, ble_store_config_csfcs, ble_store_config_num_csfcs);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for CSFC");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting CSFC, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_CSFC, nvs_count);
	}
	return 0;
}
#endif

#if MYNEWT_VAL(ENC_ADV_DATA)
int ble_store_persist_eads(void)
{
	union ble_store_value val;
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, 0, NULL, 0);
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "NVS operation failed while persisting EAD");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}

	if (nvs_count < ble_store_config_num_eads) {
		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting EAD value in NVS...");
		val.ead = ble_store_config_eads[ble_store_config_num_eads - 1];
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, &val);
	} else if (nvs_count > ble_store_config_num_eads) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, 0, ble_store_config_eads, ble_store_config_num_eads);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for EAD");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting EAD, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, nvs_count);
	}
	return 0;
}
#endif
int ble_store_persist_local_irk(void)
{
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_LOCAL_IRK, 0, NULL, 0);
	if (nvs_count == -1) {
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	int has = *(uint64_t*)&ble_store_local_irk[0];
	if (nvs_count < 1 && has) {
		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting Local IRK value in NVS...");
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_LOCAL_IRK, &ble_store_local_irk[0]);
	} else if (nvs_count > 0 && !has) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_LOCAL_IRK, 0, ble_store_local_irk, 1);
		if (nvs_count == -1) {
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting Local IRK, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_LOCAL_IRK, nvs_count);
	}
	return 0;
}

int ble_store_persist_rpa_recs(void)
{/*
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_ADDR, 0, NULL, 0);
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "NVS operation failed while persisting RPA_RECS");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_rpa_recs) {
		// NVS db count less than RAM count, write operation
		ESP_LOGD(TAG, "Persisting RPA_RECS value in NVS...");
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_PEER_ADDR, &ble_store_config_rpa_recs[ble_store_config_num_rpa_recs - 1]);
	} else if (nvs_count > ble_store_config_num_rpa_recs) {
		//NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_ADDR, 0,
									   ble_store_config_rpa_recs, ble_store_config_num_rpa_recs);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for RPA_REC");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting RPA_REC, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_PEER_ADDR, nvs_count);
	}*/
	return 0;
}

int ble_store_persist_peer_secs(void)
{
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_SEC, 0, NULL, 0);
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "operation failed while persisting peer sec");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_num_peer_secs) {
		// NVS db count less than RAM count, write operation 
		ESP_LOGI(TAG, "Persisting peer sec value...");
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_PEER_SEC, &ble_store_peer_secs[ble_store_num_peer_secs - 1]);
	} else if (nvs_count > ble_store_num_peer_secs) {
		// NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_SEC, 0, ble_store_peer_secs, ble_store_num_peer_secs);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "delete operation failed for peer sec");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGI(TAG, "Deleting peer sec, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_PEER_SEC, nvs_count);
	}
	return 0;
}

int ble_store_persist_our_secs(void)
{/*
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_OUR_SEC, 0, NULL, 0);
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "NVS operation failed while persisting our sec");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}
	if (nvs_count < ble_store_config_num_our_secs) {
		// NVS db count less than RAM count, write operation
		ESP_LOGD(TAG, "Persisting our sec value to NVS...");
		return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_OUR_SEC, &ble_store_config_our_secs[ble_store_config_num_our_secs - 1]);
	} else if (nvs_count > ble_store_config_num_our_secs) {
		//NVS db count more than RAM count, delete operation
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_OUR_SEC, 0, ble_store_config_our_secs, ble_store_config_num_our_secs);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for our sec");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting our sec, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_OUR_SEC, nvs_count);
	}*/
	return 0;
}


#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
int ble_store_persist_peer_records(void)
{
	
	int ble_store_num_peer_dev_rec = ble_rpa_get_num_peer_dev_records();
	struct ble_hs_dev_records* peer_dev_rec = ble_rpa_get_peer_dev_records();
	if (peer_dev_rec == NULL) {
		return BLE_HS_ENOMEM;
	}
	int nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, 0, NULL, 0);
	if (nvs_count == -1) {
		ESP_LOGE(TAG, "NVS operation failed while persisting peer_dev_rec");
		BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
		return BLE_HS_ESTORE_FAIL;
	}

	if (nvs_count < ble_store_num_peer_dev_rec) {

		/* NVS db count less than RAM count, write operation */
		ESP_LOGD(TAG, "Persisting peer dev record to NVS...");
		return ble_store_nvs_peer_records(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, &peer_dev_rec[ble_store_num_peer_dev_rec - 1]);
	} else if (nvs_count > ble_store_num_peer_dev_rec) {
		/* NVS db count more than RAM count, delete operation */
		nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, 0, peer_dev_rec, ble_store_num_peer_dev_rec);
		if (nvs_count == -1) {
			ESP_LOGE(TAG, "NVS delete operation failed for peer records");
			BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
			return BLE_HS_ESTORE_FAIL;
		}
		ESP_LOGD(TAG, "Deleting peer record, idx = %d", nvs_count);
		return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, nvs_count);
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
