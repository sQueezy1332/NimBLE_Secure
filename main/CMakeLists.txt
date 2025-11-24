file(GLOB_RECURSE srcs [FOLLOW_SYMLINKS] "src/*.c" "src/*.cpp"
    "libs/ESPAsyncWebServer/*.cpp" 
    "libs/AsyncTCP/*.cpp")

idf_component_register(SRCS "${srcs}" SRCS "main.cpp" 
    INCLUDE_DIRS "./include" "./libs"
    "libs/AsyncTCP"
    "libs/ESPAsyncWebServer"
    "libs/ESP_OTAserver"
    EMBED_FILES "data/config.html.gz" "data/ota.html.gz"
    ) 
set(ENV{ARDUINO_SKIP_TICK_CHECK} true)
