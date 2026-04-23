file(GLOB_RECURSE srcs [FOLLOW_SYMLINKS] "src/*.c" "src/*.cpp"
    #"libs/ESPAsyncWebServer/*.cpp" 
    #"libs/AsyncTCP/*.cpp"
    )

idf_component_register(SRCS "${srcs}" SRCS "main.cpp" 
    INCLUDE_DIRS "./include" #"./libs"
    #"libs/AsyncTCP"
    #"libs/ESPAsyncWebServer" 
	#"libs/ESP_OTAserver"
    EMBED_FILES "data/config.html.gz"
    "data/index.html.gz"#"data/ota.html.gz" 
    ) 
#spiffs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)
# set(ENV{ARDUINO_SKIP_TICK_CHECK} true)
