idf_component_register(SRCS "src/ESP_MAIN.c"
						INCLUDE_DIRS "."
						#PRIV_REQUIRES esp_driver_gpio esp_driver_gptimer
						REQUIRES nvs_flash app_update esp_driver_gptimer esp_driver_gpio esp_timer 
						esp_hal_wdt
					)