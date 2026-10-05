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

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int ble_store_config_read_hook(int obj_type, const union ble_store_key *key, union ble_store_value *value);
int ble_store_config_write_hook(int obj_type, const union ble_store_value *val);
int ble_store_config_delete_hook(int obj_type, const union ble_store_key *key);
void ble_store_init(void);
void print_bond_data(const struct ble_store_value_sec *val);

typedef struct ble_store_nvs {
    ble_addr_t peer_addr;
	uint64_t rand_num;
	uint8_t ltk[16];
	uint8_t irk[16];
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
   	uint16_t bond_count;
#endif
    uint16_t ediv;
    uint8_t key_size;
	uint8_t ltk_present:1;
	uint8_t irk_present:1;
    //uint8_t csrk[16];
	//uint8_t csrk_present:1;
	uint8_t authenticated:1; //unsigned
	uint8_t sc:1;
	//uint32_t sign_counter; //without enc
} ble_store_t;
static_assert(sizeof(ble_store_t) == 56);

#ifdef __cplusplus
}
#endif

