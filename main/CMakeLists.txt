file(GLOB_RECURSE srcs [FOLLOW_SYMLINKS] "src/*.c" "src/*.cpp" 
	"libs/nimble/src/*.c"
	"libs/wifi/*.cpp"
	"libs/misc/*.cpp" "libs/misc/*.c"
	"libs/httpd/*.cpp"
    )

idf_component_register(SRCS "${srcs}" "main.cpp"
    INCLUDE_DIRS "./include"  
	"./libs/nimble/inc"
	"./libs/wifi/"
	"./libs/misc/"
    EMBED_FILES "files/config.html.gz"
   "files/ota.html.gz" 
    ) 
