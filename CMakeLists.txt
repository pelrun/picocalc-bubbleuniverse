# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(bubble C CXX ASM)

pico_sdk_init()

add_subdirectory(lcdspi)
add_subdirectory(i2ckbd)

add_executable(bubble
  main.c
)

target_link_libraries(bubble
  pico_stdlib
  pico_multicore
  lcdspi
  i2ckbd
)

pico_add_extra_outputs(bubble)

#pico_set_binary_type(bubble no_flash)

target_link_options(bubble PRIVATE -Wl,--print-memory-usage)
