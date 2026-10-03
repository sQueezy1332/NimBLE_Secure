idf_component_register(SRCS "src/http_server.cpp"
                       PRIV_REQUIRES esp_http_server app_update esp_timer
					   EMBED_FILES "${PROJECT_DIR}/files/config.html.gz" "${PROJECT_DIR}/files/ota.html.gz" 
                       #INCLUDE_DIRS "."
					   )
idf_component_get_property(dir esp_http_server COMPONENT_DIR)
target_include_directories(${COMPONENT_TARGET} PRIVATE 
		"${dir}/src"
		"${dir}/src/port/esp32"
)
