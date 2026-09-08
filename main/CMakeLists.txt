file(GLOB_RECURSE srcs [FOLLOW_SYMLINKS] "src/*.c" "src/*.cpp"
    )

idf_component_register(SRCS "${srcs}" "main.cpp"
    INCLUDE_DIRS "./include" #"./libs"
    EMBED_FILES "files/config.html.gz"
   "files/ota.html.gz" 
    ) 
