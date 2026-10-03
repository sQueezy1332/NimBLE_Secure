idf_component_register(SRCS "src/gap.c" 			
						"src/gatt_cts_svc.c" 
						"src/gatt_hid_svc.c"
						"src/gatt.c"
						PRIV_REQUIRES bt
                     	INCLUDE_DIRS "inc"
						#"libs/nimble/storage/inc"
					   )