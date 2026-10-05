idf_component_register(SRCS "src/wifi_api.cpp"
						REQUIRES esp_wifi esp_netif
                     	INCLUDE_DIRS "."
					   )
