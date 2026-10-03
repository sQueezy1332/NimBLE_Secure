# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
#set(EXTRA_COMPONENT_DIRS "C:/.../")
#idf_build_set_property(COMPILE_OPTIONS "-fsanitize=undefined" "-fno-sanitize=shift-base" APPEND)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(NimBLE_Secure)