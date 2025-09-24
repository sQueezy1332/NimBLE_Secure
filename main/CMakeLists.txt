file(GLOB_RECURSE srcs "main.cpp" "src/*.c" "src/*.cpp" "include/*.h")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "./include")
#spiffs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)
set(ENV{ARDUINO_SKIP_TICK_CHECK} true)