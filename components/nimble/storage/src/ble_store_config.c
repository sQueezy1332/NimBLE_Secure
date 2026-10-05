/*
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

#include <inttypes.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "ble_store_config.h"
#include "esp_nimble_mem.h"
//#include "../src/ble_hs_priv.h" //BLE_HS_DBG_ASSERT
#include "host/ble_hs_log.h"
#include "ble_store_config_priv.h"
#include "rom/crc.h"
#include "common.h"

#pragma GCC diagnostic ignored "-Wunused-function"

#define BONDS_ATTR __NOINIT_ATTR
#if MYNEWT_VAL(BLE_HS_DEBUG)
    #define BLE_HS_DBG_ASSERT(x) assert(x)
    #define BLE_HS_DBG_ASSERT_EVAL(x) assert(x)
#else
    #define BLE_HS_DBG_ASSERT(x)
    #define BLE_HS_DBG_ASSERT_EVAL(x) ((void)(x))
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
ble_store_config_vars_t * ble_store_config_vars = NULL;
#endif
static const char *TAG = "BLE_STORE";

#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
ble_store_t ble_store_our_secs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
//int ble_store_num_our_secs;
//uint16_t ble_store_config_our_bond_count;
#endif

BONDS_ATTR uint16_t ble_store_peer_bond_count;
BONDS_ATTR uint16_t crc_bonds;

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
BONDS_ATTR ble_store_t ble_store_peer_secs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
BONDS_ATTR int ble_store_num_peer_secs;
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
//struct ble_store_value_cccd ble_store_config_cccds[MYNEWT_VAL(BLE_STORE_MAX_CCCDS)];
//int ble_store_config_num_cccds;
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
//struct ble_store_value_csfc ble_store_config_csfcs[MYNEWT_VAL(BLE_STORE_MAX_CSFCS)];
//int ble_store_config_num_csfcs;
#endif

#if MYNEWT_VAL(ENC_ADV_DATA)
//struct ble_store_value_ead ble_store_config_eads[MYNEWT_VAL(BLE_STORE_MAX_EADS)];
//int ble_store_config_num_eads;
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
//struct ble_store_value_rpa_rec  ble_store_config_rpa_recs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
//int ble_store_config_num_rpa_recs;

BONDS_ATTR struct ble_store_value_local_irk ble_store_local_irk[1];
//int ble_store_config_num_local_irks;
#endif
#endif /* !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

//		$sec	
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
static int ble_store_config_write_peer_sec(const struct ble_store_value_sec *);
static int ble_store_config_delete_peer_sec(const struct ble_store_key_sec *);

int ble_store_compare_bond_count(const void *a, const void *b) {
    const ble_store_t *sec_a = (ble_store_t *)a;
    const ble_store_t *sec_b = (ble_store_t *)b;
    return (sec_a->bond_count > sec_b->bond_count) - (sec_a->bond_count < sec_b->bond_count);
}

/* This function gets the stored device records of OUR_SEC object type, arranges them in order of their bond count,
 * and then updates them with new counts so they're in sequence.
 */
int ble_rearrange_our_sec_nvs(void)
{
    struct ble_store_value_sec temp_our_secs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
    ble_store_config_our_bond_count = 0;
    memcpy(temp_our_secs, ble_store_config_our_secs, ble_store_config_num_our_secs * sizeof(ble_store_config_our_secs[0]));
	int err; int temp_count = ble_store_config_num_our_secs;
    qsort(temp_our_secs, temp_count, sizeof(*temp_our_secs), ble_store_compare_bond_count);
    for (int i = 0; i < temp_count; i++) {

        union ble_store_key key;
        ble_store_key_from_value_sec(&key.sec, &temp_our_secs[i]);

        err = ble_store_config_delete_hook(BLE_STORE_OBJ_TYPE_OUR_SEC, &key);

        if (err != ESP_OK) {
            BLE_HS_LOG(DEBUG, "Error deleting from nvs");
            return err;
        }
    }

    for (int i = 0; i < temp_count; i++) {
        union ble_store_value val = {.sec =  temp_our_secs[i] };
        err = ble_store_config_write_hook(BLE_STORE_OBJ_TYPE_OUR_SEC, &val);
        if (err != ESP_OK) {
            BLE_HS_LOG(DEBUG, "Error writing to nvs");
            return err;
        }
    }
    /* The global array ble_store_config_our_secs and its count are already correctly updated
     * by the ble_store_config_write calls in the loop above. Overwriting them here with
     * temp_our_secs would revert the bond_count reset. */
    return 0;
}

/* This function gets the stored device records of PEER_SEC object type, arranges them in order of their bond count,
 * and then updates them with new counts so they're in sequence.
 */
int ble_rearrange_peer_sec_nvs(void)
{
	int err; int temp_count = ble_store_num_peer_secs;
    ble_store_t *temp_secs = nimble_platform_mem_malloc(temp_count);//[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
	if(!temp_secs) {
		ESP_LOGE(TAG, "BLE_HS_ENOMEM");
		return BLE_HS_ENOMEM;
	}
	ble_store_peer_bond_count = 0;
    memcpy(temp_secs, ble_store_peer_secs, temp_count * sizeof(*temp_secs));
    qsort(temp_secs, temp_count, sizeof(*temp_secs), ble_store_compare_bond_count);
    for (size_t i = 0; i < temp_count; i++) {
        //union ble_store_key key;  ble_store_key_from_value_sec(&key.sec, &temp_secs[i]);
        err = ble_store_config_delete_peer_sec((struct ble_store_key_sec *)&temp_secs[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "deleting from nvs %d ",err);
            goto exit;
        }
    }
    for (size_t i = 0; i < temp_count; i++) {
		ble_store_peer_secs[i] = temp_secs[i];
		ble_store_peer_bond_count = ble_store_num_peer_secs = i + 1;
		ble_store_peer_secs[i].bond_count = ble_store_peer_bond_count;
    }
	crc_bonds = crc16_le(0, (uint8_t*)ble_store_peer_secs, sizeof(ble_store_peer_secs)); //TODO
	err = ble_store_config_persist_peer_secs();
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "writing to nvs %d ", err); //goto exit;
	}
    /* The global array ble_store_config_peer_secs and its count are already correctly updated
     * by the ble_store_config_write calls in the loop above. Overwriting them here with
     * temp_peer_secs would revert the bond_count reset. */
exit:	
	nimble_platform_mem_free(temp_secs);
    return err;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
static void
ble_store_config_print_value_sec(const struct ble_store_value_sec *sec)
{
    /*
     * NOTE: This function's multi-call logging pattern is intentionally preserved.
     * ESP-IDF's BLE_HS_LOG implementation may buffer or consolidate outputs.
     * The separate calls allow conditional logging of different security components
     * (LTK, IRK, CSRK) which is useful for debugging. No consolidation needed
     * unless ESP-IDF logging performance issues are observed.
     */
    if (sec->ltk_present) {
        BLE_HS_LOG(DEBUG, "ediv=%u rand=%llu authenticated=%d ltk=",
                       sec->ediv, sec->rand_num, sec->authenticated);
        ble_hs_log_flat_buf(sec->ltk, 16);
        BLE_HS_LOG(DEBUG, " ");
    }
    if (sec->irk_present) {
        BLE_HS_LOG(DEBUG, "irk=");
        ble_hs_log_flat_buf(sec->irk, 16);
        BLE_HS_LOG(DEBUG, " ");
    }
    if (sec->csrk_present) {
        BLE_HS_LOG(DEBUG, "csrk=");
        ble_hs_log_flat_buf(sec->csrk, 16);
        BLE_HS_LOG(DEBUG, " sign_counter = %u", sec->sign_counter);
    }

    BLE_HS_LOG(DEBUG, "\n");
}
#endif

static void
ble_store_config_print_key_sec(const struct ble_store_key_sec *key_sec)
{
    if (ble_addr_cmp(&key_sec->peer_addr, BLE_ADDR_ANY) != 0) { 
        BLE_HS_LOG(DEBUG, "peer_addr_type=%d peer_addr=", key_sec->peer_addr.type);
        ble_hs_log_flat_buf(key_sec->peer_addr.val, 6);
        BLE_HS_LOG(DEBUG, " ");
    }
}

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)

static int 
ble_store_config_find_sec(const struct ble_store_key_sec *key_sec, const ble_store_t *value_secs, int num_value_secs)
{
	if (ble_addr_cmp(&key_sec->peer_addr, BLE_ADDR_ANY) == 0) {  // != ANY
		uint8_t idx = key_sec->idx;
		if (idx < num_value_secs) 
			return idx;
	}
    for (int i = 0; i < num_value_secs; i++) {
        if (ble_addr_cmp(&value_secs[i].peer_addr, &key_sec->peer_addr) == 0) {
			return i;
		}
    }
    return -1;
}
#endif
static int ble_store_config_read_peer_sec(const struct ble_store_key_sec *, struct ble_store_value_sec *);
static int
ble_store_config_read_our_sec(const struct ble_store_key_sec *key_sec, struct ble_store_value_sec *value_sec)
{	
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	int rc = ble_store_config_read_peer_sec(key_sec, value_sec);
	if(rc) return rc;
	if (likely(*(uint64_t*)&ble_store_local_irk)) {
		memcpy(value_sec->irk, ble_store_local_irk[0].irk, sizeof(ble_store_local_irk[0].irk));
		value_sec->irk_present = 1;
	} else { 
		value_sec->irk_present = 0; 
	}
	return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

static int
ble_store_config_write_our_sec(const struct ble_store_value_sec *value_sec)
{return 0;
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    struct ble_store_key_sec key_sec;
    int idx; int rc;
    BLE_HS_LOG(DEBUG, "persisting our sec; ");
    ble_store_config_print_value_sec(value_sec);
    ble_store_key_from_value_sec(&key_sec, value_sec);
    idx = ble_store_config_find_sec(&key_sec, ble_store_config_our_secs, ble_store_config_num_our_secs);
    if (idx == -1) {
        if (ble_store_config_num_our_secs >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
            BLE_HS_LOG(DEBUG, "error persisting our sec; too many entries "
                              "(%d)\n", ble_store_config_num_our_secs);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }
        idx = ble_store_config_num_our_secs++;
    }
    //ble_store_config_our_secs[idx] = *value_sec;
    ble_store_config_our_secs[idx].bond_count = ++ble_store_config_our_bond_count;
    rc = ble_store_config_persist_our_secs();
    if (rc != 0) {
        return rc;
    }
    if (ble_store_config_our_bond_count > (UINT16_MAX - 5)) {
        rc = ble_rearrange_our_sec_nvs();
        if (rc != 0) {
            return rc;
        }
    }
    return 0;
#else
    return BLE_HS_ENOENT;
#endif

}

static int ble_store_config_delete_obj(void *values, int value_size, int idx, int *num_values)
{
    BLE_HS_DBG_ASSERT(idx >= 0 && idx < *num_values && *num_values > 0);
    (*num_values)--;
    if (idx < *num_values) {
        uint8_t *dst = values + (idx * value_size);
        uint8_t *src = dst + value_size;
        size_t move_count = *num_values - idx;
        memmove(dst, src, move_count * value_size);
    }
    return 0;
}

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
static int
ble_store_config_delete_sec(const struct ble_store_key_sec *key_sec, ble_store_t *value_secs, int *num_value_secs)
{
    int idx = ble_store_config_find_sec(key_sec, value_secs, *num_value_secs);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }
    return ble_store_config_delete_obj(value_secs, sizeof *value_secs, idx, num_value_secs);
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
static int ble_store_config_delete_our_sec(const struct ble_store_key_sec *key_sec)
{
	return BLE_HS_ENOENT;
	//return ble_store_config_delete_peer_sec(key_sec);
}

static int ble_store_config_delete_peer_sec(const struct ble_store_key_sec *key_sec)
{
	int rc = ble_store_config_delete_sec(key_sec, ble_store_peer_secs, &ble_store_num_peer_secs);
    if (rc == 0) {
		rc = ble_store_config_persist_peer_secs();
		crc_bonds = crc16_le(0, (uint8_t*)ble_store_peer_secs, sizeof(ble_store_peer_secs)); //TODO
    }
    return rc;
}
#endif

static int 
ble_store_config_read_peer_sec(const struct ble_store_key_sec *key_sec, struct ble_store_value_sec *value_sec)
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    int idx = ble_store_config_find_sec(key_sec, ble_store_peer_secs, ble_store_num_peer_secs);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }
	const ble_store_t *bond = &ble_store_peer_secs[idx];
    value_sec->peer_addr = bond->peer_addr;
	value_sec->bond_count = bond->bond_count;
	value_sec->key_size = bond->key_size;
	value_sec->ediv = bond->ediv;
	value_sec->rand_num = bond->rand_num;
	value_sec->sign_counter = 0;
	value_sec->ltk_present = bond->ltk_present;
	value_sec->irk_present = bond->irk_present;
	value_sec->csrk_present = 0;
	value_sec->authenticated = bond->authenticated;
	value_sec->sc = bond->sc;
	memcpy(value_sec->ltk, bond->ltk, sizeof(bond->ltk));
	memcpy(value_sec->irk, bond->irk, sizeof(bond->irk));
    return 0;
#else
    return BLE_HS_ENOENT;
#endif

}

static int ble_store_config_write_peer_sec(const struct ble_store_value_sec *value_sec)
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    print_bond_data(value_sec);
	//struct ble_store_key_sec key_sec; ble_store_key_from_value_sec(&key_sec, value_sec);
    int rc, idx = ble_store_config_find_sec((struct ble_store_key_sec *)value_sec, ble_store_peer_secs, ble_store_num_peer_secs);
    if (idx == -1) {
        if (ble_store_num_peer_secs >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
            BLE_HS_LOG(DEBUG, "error persisting peer sec; too many entries "
                             "(%d)\n", ble_store_num_peer_secs);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }
        idx = ble_store_num_peer_secs++;
		ESP_LOGI(TAG, "new peer № %u", ble_store_num_peer_secs);
    }
	ble_store_t *bond = &ble_store_peer_secs[idx];
    bond->peer_addr = value_sec->peer_addr;
	bond->key_size = value_sec->key_size;
	bond->ediv = value_sec->ediv;
	bond->rand_num = value_sec->rand_num;
	bond->bond_count = value_sec->bond_count;
	bond->ltk_present = value_sec->ltk_present;
	bond->irk_present = value_sec->irk_present;
	bond->authenticated = value_sec->authenticated;
    bond->sc = value_sec->sc;
	bond->bond_count = ++ble_store_peer_bond_count;
	memcpy(bond->ltk, value_sec->ltk, sizeof(bond->ltk));
	memcpy(bond->irk, value_sec->irk, sizeof(bond->irk));
    /* Ensure entries are sorted at all times */
    //qsort(ble_store_peer_secs, ble_store_num_peer_secs, sizeof(ble_store_peer_secs[0]), ble_store_compare_bond_count);
	if (unlikely(ble_store_peer_bond_count > (UINT16_MAX - 5))) {
        rc = ble_rearrange_peer_sec_nvs();
    } else { rc = ble_store_config_persist_peer_secs(); }; 
	crc_bonds = crc16_le(0, (uint8_t*)ble_store_peer_secs, sizeof(ble_store_peer_secs)); //TODO
    return rc;
#else
    return BLE_HS_ENOENT;
#endif
}

//			$cccd	

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
static int ble_store_config_find_cccd(const struct ble_store_key_cccd *key)
{
    struct ble_store_value_cccd *cccd;
    int skipped = 0;
    for (int i = 0; i < ble_store_config_num_cccds; i++) {
        cccd = ble_store_config_cccds + i;

        if (ble_addr_cmp(&key->peer_addr, BLE_ADDR_ANY)) {
            if (ble_addr_cmp(&cccd->peer_addr, &key->peer_addr)) {
                continue;
            }
        }

        if (key->chr_val_handle != 0) {
            if (cccd->chr_val_handle != key->chr_val_handle) {
                continue;
            }
        }

        if (key->idx > skipped) {
            skipped++;
            continue;
        }

        return i;
    }
    return -1;
}
#endif

static int ble_store_config_delete_cccd(const struct ble_store_key_cccd *key_cccd)
{
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
    int idx; int rc;

    idx = ble_store_config_find_cccd(key_cccd);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }

    rc = ble_store_config_delete_obj(ble_store_config_cccds,
                                     sizeof *ble_store_config_cccds,
                                     idx,
                                     &ble_store_config_num_cccds);
    if (rc != 0) {
        return rc;
    }

    rc = ble_store_config_persist_cccds();
    if (rc != 0) {
        return rc;
    }
    return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

static int ble_store_config_read_cccd(const struct ble_store_key_cccd *key_cccd, struct ble_store_value_cccd *value_cccd)
{
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
    int idx = ble_store_config_find_cccd(key_cccd);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }
    *value_cccd = ble_store_config_cccds[idx];
    return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

static int ble_store_config_write_cccd(const struct ble_store_value_cccd *value_cccd)
{
#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
    struct ble_store_key_cccd key_cccd;
    int idx;int rc;

    ble_store_key_from_value_cccd(&key_cccd, value_cccd);
    idx = ble_store_config_find_cccd(&key_cccd);
    if (idx == -1) {
        if (ble_store_config_num_cccds >= MYNEWT_VAL(BLE_STORE_MAX_CCCDS)) {
            BLE_HS_LOG(DEBUG, "error persisting cccd; too many entries (%d)\n",
                       ble_store_config_num_cccds);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }

        idx = ble_store_config_num_cccds;
        ble_store_config_num_cccds++;
    }

    ble_store_config_cccds[idx] = *value_cccd;

    rc = ble_store_config_persist_cccds();
    if (rc != 0) {
        return rc;
    }

    return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

//			$ead 
#if MYNEWT_VAL(ENC_ADV_DATA)
static int
ble_store_config_find_ead(const struct ble_store_key_ead *key)
{
    struct ble_store_value_ead *ead;
    int skipped = 0;
    for (int i = 0; i < ble_store_config_num_eads; i++) {
        ead = ble_store_config_eads + i;

        if (ble_addr_cmp(&key->peer_addr, BLE_ADDR_ANY)) {
            if (ble_addr_cmp(&ead->peer_addr, &key->peer_addr)) {
                continue;
            }
        }

        if (key->idx > skipped) {
            skipped++;
            continue;
        }

        return i;
    }

    return -1;
}

static int
ble_store_config_delete_ead(const struct ble_store_key_ead *key_ead)
{
    int rc;
    int idx = ble_store_config_find_ead(key_ead);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }

    rc = ble_store_config_delete_obj(ble_store_config_eads,
                                     sizeof *ble_store_config_eads,
                                     idx,
                                     &ble_store_config_num_eads);
    if (rc != 0) {
        return rc;
    }

    rc = ble_store_config_persist_eads();
    if (rc != 0) {
        return rc;
    }

    return 0;
}

static int
ble_store_config_read_ead(const struct ble_store_key_ead *key_ead,
                           struct ble_store_value_ead *value_ead)
{
    int idx = ble_store_config_find_ead(key_ead);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }
    *value_ead = ble_store_config_eads[idx];
    return 0;
}

static int
ble_store_config_write_ead(const struct ble_store_value_ead *value_ead)
{
    struct ble_store_key_ead key_ead;
    ble_store_key_from_value_ead(&key_ead, value_ead);
    int idx = ble_store_config_find_ead(&key_ead);
    if (idx == -1) {
        if (ble_store_config_num_eads >= MYNEWT_VAL(BLE_STORE_MAX_EADS)) {
            BLE_HS_LOG(DEBUG, "error persisting ead; too many entries (%d)\n",
                       ble_store_config_num_eads);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }

        idx = ble_store_config_num_eads;
        ble_store_config_num_eads++;
    }
    ble_store_config_eads[idx] = *value_ead;
    int rc = ble_store_config_persist_eads();
    if (rc != 0) {
        return rc;
    }

    return 0;
}
#endif

//			$local irk
//#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
static int ble_store_config_find_local_irk(const struct ble_store_key_local_irk *key)
{
	if (key->idx != 0) return -1;
	if (!ble_addr_cmp(&key->addr, BLE_ADDR_ANY)) {
		if (likely(*(uint64_t*)&ble_store_local_irk))
			return 0;
	} else if (!ble_addr_cmp(&ble_store_local_irk[0].addr, &key->addr))
		return 0;
	return -1;
}
//#endif

static int ble_store_config_delete_local_irk(const struct ble_store_key_local_irk *key_irk)
{
//#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    int rc = ble_store_config_find_local_irk(key_irk);
    if (rc == -1) return BLE_HS_ENOENT;
	memset(&ble_store_local_irk, 0 , sizeof(ble_store_local_irk));
    rc = ble_store_config_persist_local_irk();
    return rc;
//#else
    return BLE_HS_ENOTSUP;
//#endif
}

static int ble_store_config_read_local_irk(const struct ble_store_key_local_irk *key_irk, struct ble_store_value_local_irk *value_irk)
{
//#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    int idx = ble_store_config_find_local_irk(key_irk);
    if (idx == -1) return BLE_HS_ENOENT;
    *value_irk = ble_store_local_irk[0];
    return 0;
//#else
    return BLE_HS_ENOENT;
//#endif
}

static int ble_store_config_write_local_irk(const struct ble_store_value_local_irk *value_irk)
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    struct ble_store_value_local_irk old_val = ble_store_local_irk[0];
    ble_store_local_irk[0] = *value_irk;
    int rc = ble_store_config_persist_local_irk();
    if (rc != 0) { ble_store_local_irk[0] = old_val; } 
    return rc;
#else
    return BLE_HS_ENOTSUP;
#endif
}

//			$rpa-map  

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
static int ble_store_config_find_rpa_rec(const struct ble_store_key_rpa_rec *key)
{
    struct ble_store_value_rpa_rec *rpa_rec;
    int skipped = 0;
    int i = 0;

    for(i = 0; i < ble_store_config_num_rpa_recs; i++){
        rpa_rec = ble_store_config_rpa_recs + i;

        if (ble_addr_cmp(&rpa_rec->peer_rpa_addr, &key->peer_rpa_addr) &&
            ble_addr_cmp(&rpa_rec->peer_addr, &key->peer_rpa_addr)) {
            continue;
        }
        if (key->idx > skipped) {
            skipped++;
            continue;
        }
        return i;
    }
    return -1;
}
#endif

static int ble_store_config_read_rpa_rec(const struct ble_store_key_rpa_rec *key_rpa_rec,struct ble_store_value_rpa_rec *value_rpa_rec)
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    int idx = ble_store_config_find_rpa_rec(key_rpa_rec);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }
    *value_rpa_rec = ble_store_config_rpa_recs[idx];
    return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

static int ble_store_config_write_rpa_rec(const struct ble_store_value_rpa_rec *value_rpa_rec) 
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    struct ble_store_key_rpa_rec key_rpa_rec;
    ble_store_key_from_value_rpa_rec(&key_rpa_rec, value_rpa_rec);
    int idx = ble_store_config_find_rpa_rec(&key_rpa_rec);
    if (idx == -1) {
        if (ble_store_config_num_rpa_recs >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
            BLE_HS_LOG(DEBUG, "error persisting peer addrr; too many entries (%d)\n",
                       ble_store_config_num_rpa_recs);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }

        idx = ble_store_config_num_rpa_recs;
        ble_store_config_num_rpa_recs++;
    }
    ble_store_config_rpa_recs[idx] = *value_rpa_rec;
    int rc = ble_store_config_persist_rpa_recs();
    if (rc != 0) {
        return rc;
    }
    return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

static int ble_store_config_delete_rpa_rec(const struct ble_store_key_rpa_rec *key_rpa_rec)
{
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    int idx = ble_store_config_find_rpa_rec(key_rpa_rec);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }

    int rc = ble_store_config_delete_obj(ble_store_config_rpa_recs,
                                     sizeof *ble_store_config_rpa_recs,
                                     idx,
                                     &ble_store_config_num_rpa_recs);
    if (rc != 0) { return rc; }

    rc = ble_store_config_persist_rpa_recs();
    return rc;
#else
    return BLE_HS_ENOENT;
#endif
}

//			$csfc
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
static int ble_store_config_find_csfc(const struct ble_store_key_csfc *key, const struct ble_store_value_csfc *value_csfc, int num_value_csfc)
{
    const struct ble_store_value_csfc *cur;
    int i;

    // If peer_addr is specified, search by peer_addr (common for write/read/delete) 
    if (ble_addr_cmp(&key->peer_addr, BLE_ADDR_ANY)) {
        for (i = 0; i < num_value_csfc; i++) {
            cur = &value_csfc[i];
            if (!ble_addr_cmp(&cur->peer_addr, &key->peer_addr)) {
                return i;
            }
        }
    } else {
        // For ANY peer use idx as direct array index (enumerate scenario) 
        if (key->idx < num_value_csfc) {
            return key->idx;
        }
    }
    return -1;
}
#endif

static int ble_store_config_delete_csfc(const struct ble_store_key_csfc *key_csfc)
{
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
    int idx = ble_store_config_find_csfc(key_csfc, ble_store_config_csfcs,
                                     ble_store_config_num_csfcs);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }

    int rc = ble_store_config_delete_obj(ble_store_config_csfcs,
                                     sizeof *ble_store_config_csfcs,
                                     idx, &ble_store_config_num_csfcs);

    if (rc != 0) {
        return rc;
    }

    rc = ble_store_config_persist_csfcs();
    return rc;
#else
    return BLE_HS_ENOENT;
#endif
}

static int ble_store_config_read_csfc(const struct ble_store_key_csfc *key_csfc, struct ble_store_value_csfc *value_csfc)
{
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
    int idx = ble_store_config_find_csfc(key_csfc, ble_store_config_csfcs, ble_store_config_num_csfcs);
    if (idx == -1) {
        return BLE_HS_ENOENT;
    }
    *value_csfc = ble_store_config_csfcs[idx];
    return 0;
#else
    return BLE_HS_ENOENT;
#endif
}

static int ble_store_config_write_csfc(const struct ble_store_value_csfc *value_csfc)
{
#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
    struct ble_store_key_csfc key_csfc;
    ble_store_key_from_value_csfc(&key_csfc, value_csfc);
    int idx = ble_store_config_find_csfc(&key_csfc, ble_store_config_csfcs,
                                     ble_store_config_num_csfcs);
    if (idx == -1) {
        if (ble_store_config_num_csfcs >= MYNEWT_VAL(BLE_STORE_MAX_CSFCS)) {
            BLE_HS_LOG(DEBUG, "error persisting csfc; too many entries (%d)\n",
                       ble_store_config_num_csfcs);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }

        idx = ble_store_config_num_csfcs;
        ble_store_config_num_csfcs++;
    }

    ble_store_config_csfcs[idx] = *value_csfc;

    int rc = ble_store_config_persist_csfcs();
	return rc;
#else
    return BLE_HS_ENOENT;
#endif
}

//			$api
/**
 * Searches the database for an object matching the specified criteria.
 *
 * @return                      0 if a key was found; else BLE_HS_ENOENT.
 */
int ble_store_config_read_hook(int obj_type, const union ble_store_key* key, union ble_store_value* value) {
	int rc;
	switch (obj_type) {
	case BLE_STORE_OBJ_TYPE_PEER_SEC: //BLE_HS_LOG(DEBUG, "looking up peer sec; "); ble_store_config_print_key_sec(&key->sec); BLE_HS_LOG(DEBUG, "\n");
		rc = ble_store_config_read_peer_sec(&key->sec, &value->sec); break;
	case BLE_STORE_OBJ_TYPE_OUR_SEC: //BLE_HS_LOG(DEBUG, "looking up our sec; "); ble_store_config_print_key_sec(&key->sec); BLE_HS_LOG(DEBUG, "\n");
		rc = ble_store_config_read_our_sec(&key->sec, &value->sec); break;
	//case BLE_STORE_OBJ_TYPE_CCCD: return rc = ble_store_config_read_cccd(&key->cccd, &value->cccd);
	//case BLE_STORE_OBJ_TYPE_CSFC: return rc = ble_store_config_read_csfc(&key->csfc, &value->csfc);
#if MYNEWT_VAL(ENC_ADV_DATA)
	case BLE_STORE_OBJ_TYPE_ENC_ADV_DATA:
		rc =  ble_store_config_read_ead(&key->ead, &value->ead); break;
#endif 
	//case BLE_STORE_OBJ_TYPE_PEER_ADDR: return rc = ble_store_config_read_rpa_rec(&key->rpa_rec, &value->rpa_rec);
	case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
		rc = ble_store_config_read_local_irk(&key->local_irk, &value->local_irk); break;
	default: rc = BLE_HS_ENOTSUP;
	} ESP_LOGI(TAG, "%s obj_type %u %s", "read", 
		obj_type, rc == BLE_HS_ENOTSUP ? "\tENOTSUP" : rc == 0 ? "+" : "-");
	return rc;
}

/**
 * Adds the specified object to the database.
 *
 * @return                      0 on success;
 *                              BLE_HS_ESTORE_CAP if the database is full.
 */
int ble_store_config_write_hook(int obj_type, const union ble_store_value* val) {
	int rc;
	switch (obj_type) {
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	case BLE_STORE_OBJ_TYPE_PEER_SEC:
		rc = ble_store_config_write_peer_sec(&val->sec); break;
	case BLE_STORE_OBJ_TYPE_OUR_SEC: print_bond_data(&val->sec); rc = BLE_HS_ENOTSUP; break;
#endif
	//case BLE_STORE_OBJ_TYPE_CCCD: return rc = ble_store_config_write_cccd(&val->cccd);
	//case BLE_STORE_OBJ_TYPE_CSFC:return rc = ble_store_config_write_csfc(&val->csfc);
#if MYNEWT_VAL(ENC_ADV_DATA)
	case BLE_STORE_OBJ_TYPE_ENC_ADV_DATA:
		rc = ble_store_config_write_ead(&val->ead); break;
#endif
	//case BLE_STORE_OBJ_TYPE_PEER_ADDR:return rc = ble_store_config_write_rpa_rec(&val->rpa_rec);
	case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
		rc = ble_store_config_write_local_irk(&val->local_irk); break;
	default: rc = BLE_HS_ENOTSUP;
	} ESP_LOGI(TAG, "%s obj_type %u %s", "write", 
		obj_type, rc == BLE_HS_ENOTSUP ? "\tENOTSUP" : rc == 0 ? "+" : "-");
	return rc;
}

int ble_store_config_delete_hook(int obj_type, const union ble_store_key *key) {
	int rc;
	switch (obj_type) {
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
        rc = ble_store_config_delete_peer_sec(&key->sec); break;
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
        rc = ble_store_config_delete_our_sec(&key->sec); break;
#endif
    //case BLE_STORE_OBJ_TYPE_CCCD: return ble_store_config_delete_cccd(&key->cccd);
    //case BLE_STORE_OBJ_TYPE_CSFC: return ble_store_config_delete_csfc(&key->csfc);   
#if MYNEWT_VAL(ENC_ADV_DATA)
    case BLE_STORE_OBJ_TYPE_ENC_ADV_DATA: 
		rc =  ble_store_config_delete_ead(&key->ead); break;
#endif
    //case BLE_STORE_OBJ_TYPE_PEER_ADDR: return ble_store_config_delete_rpa_rec(&key->rpa_rec);
	case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
        rc = ble_store_config_delete_local_irk(&key->local_irk); break;
    default: rc = BLE_HS_ENOTSUP;
	} ESP_LOGI(TAG, "%s obj_type %u %s", "delete", 
		obj_type, rc == BLE_HS_ENOTSUP ? "\tENOTSUP" : rc == 0 ? "+" : "-");
	return rc;
}

void ble_store_init(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_store_config_vars == NULL) {
        ble_store_config_vars = nimble_platform_mem_calloc(1, sizeof(ble_store_config_vars_t));
        if (ble_store_config_vars == NULL) {
            MODLOG_DFLT(ERROR, "Failed to allocate memory for ble_store_config_vars\n");
            assert(0);
            return;
        }
    }
#endif
    SYSINIT_ASSERT_ACTIVE(); // Ensure this function only gets called by sysinit.
    ble_hs_cfg.store_read_cb = ble_store_config_read_hook;
	ble_hs_cfg.store_write_cb = ble_store_config_write_hook;
	ble_hs_cfg.store_delete_cb = ble_store_config_delete_hook;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	//TODO
	if(crc_bonds != crc16_le(0, (uint8_t*)&ble_store_peer_secs, sizeof(ble_store_peer_secs))) {
		ESP_LOGW(TAG, "!crc_bonds");
		memset(ble_store_peer_secs, 0, sizeof(ble_store_peer_secs));
		memset(&ble_store_local_irk, 0, sizeof(ble_store_local_irk)); 
		ble_store_num_peer_secs = 0; 
		ble_store_peer_bond_count = 0;
		crc_bonds = crc16_le(0, (uint8_t*)ble_store_peer_secs, sizeof(ble_store_peer_secs));
	}
    //ble_store_nvs_init();
}

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) //|| MYNEWT_VAL(MP_RUNTIME_ALLOC)
void
ble_store_config_deinit(void)
{
    if (ble_store_config_vars != NULL) {
        nimble_platform_mem_free(ble_store_config_vars);
        ble_store_config_vars = NULL;
    }
}
#endif

void print_bond_data(const struct ble_store_value_sec *val) {
	extern const char* format_addr(const uint8_t *addr);
	NIMLOG("peer address: %s (%u)", format_addr(val->peer_addr.val), val->peer_addr.type);
	NIMLOG("\nkey_size %u", val->key_size);
	if(val->ediv) {
		NIMLOG("\nediv %u", val->ediv);
	}
	if(val->rand_num) {
		NIMLOG("\nrand_num 0x%08X", ((size_t*)&val->rand_num)[1]); 
		NIMLOG("%08X", ((size_t*)&val->rand_num)[0]);
	}	
	if(val->ltk_present) {
		NIMLOG("\nLTK:\t");
		for (size_t i = 0; i < sizeof(val->ltk); ++i) { NIMLOG("%02X", val->ltk[i]);} 
	}
	if(val->irk_present) {
		NIMLOG("\nIRK:\t");
		for (size_t i = 0; i < sizeof(val->irk); ++i) { NIMLOG("%02X", val->irk[i]);} 
	}
	if(val->sign_counter) {
		NIMLOG("\nsign_counter %u", (size_t)val->sign_counter);
	}
	NIMLOG("\nauthenticated %u, sc %u\n", val->authenticated, val->sc);
}

/*
int ble_store_find(const struct ble_store_key_sec *key_sec, const ble_store_t* value_secs, size_t num_value_secs) {
	if(num_value_secs) {
		size_t i = key_sec->idx;
		((struct ble_store_key_sec *)key_sec)->idx = 0;
		uint64_t key = *(uint64_t*)&key_sec->peer_addr;
		if(key == 0x00) {
			ESP_LOGI(TAG, "finding BLE_ADDR_ANY key idx: %u", i);
			if(i < num_value_secs)
				return i;
			return -1;
		}
		for (i = 0; i < num_value_secs; i++) {
			const struct ble_store_value_sec* cur = &value_secs[i];
			if (key == *(uint64_t*)&(cur->peer_addr)) {
                ESP_LOGI(TAG, "finded addr [%u]: %s",i, format_addr(cur->peer_addr.val));
				return i;
            }
		}
	}
	return -1;
}*/

