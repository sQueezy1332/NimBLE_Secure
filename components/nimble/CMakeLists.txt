idf_component_register(SRCS "src/gap.c" 			
						"src/gatt_cts_svc.c" 
						"src/gatt_hid_svc.c"
						"src/gatt.c"
						"storage/src/ble_store_config.c"
						"storage/src/ble_store_nvs.c"
						PRIV_REQUIRES bt nvs_flash
                     	INCLUDE_DIRS "inc"
						INCLUDE_DIRS "storage/inc"
						PRIV_INCLUDE_DIRS "storage/src"
					   )