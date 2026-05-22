file(GLOB_RECURSE srcs [FOLLOW_SYMLINKS] "src/*.c" "src/*.cpp" "main.cpp"
    #"libs/ESPAsyncWebServer/*.cpp" 
    #"libs/AsyncTCP/*.cpp"
    )

idf_component_register(SRCS "${srcs}"
    INCLUDE_DIRS "./include" #"./libs"
    EMBED_FILES "files/config.html.gz"
   "files/ota.html.gz" 
    ) 
#spiffs_create_partition_image(spiffs ../files FLASH_IN_PROJECT)
