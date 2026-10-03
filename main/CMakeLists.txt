#file(GLOB_RECURSE MY_SRCS FOLLOW_SYMLINKS "src/*.c" "src/*.cpp" 
    #)
#SRC_DIRS "."

idf_component_register(SRCS "main.cpp"
	#"${MY_SRCS}" 
    PRIV_INCLUDE_DIRS "include" 
	PRIV_REQUIRES nimble bt httpd wifi misc led mbedtls esp_driver_usb_serial_jtag
    ) 
#idf_component_get_property(dir tlsf COMPONENT_DIR)
#target_include_directories(${COMPONENT_TARGET} PRIVATE  "${dir}/heap/tlsf" )