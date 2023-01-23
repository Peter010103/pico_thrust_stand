cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(dshot_test LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

# file(GLOB FILES *.cpp *.h)
# file(GLOB FILES ./*/*.cpp ./*/*.h)
# file(GLOB FILES ./*/*/*.cpp ./*/*/*.h)

# include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} 
#     ./lib/dshot/dshot.cpp ./lib/dshot/dshot.h
#     ./lib/shoot/shoot.cpp ./lib/shoot/shoot.h
#     ./lib/tts/tts.cpp ./lib/tts/tts.h
#     ./lib/dshot/dshot.cpp ./lib/dshot/dshot.h
# )


include_directories(./lib/shoot)
include_directories(./lib/tts)
include_directories(./lib/utils)
include_directories(./lib/dshot)
include_directories(./include)
include_directories(./include/arduino-pico/cores/rp2040)
include_directories(./include/pico-extras/lib/lwip/src/include/)
include_directories(./include/pico-extras/src/rp2_common/lwip/include)
include_directories(./include/arduino-pico/variants/rpipico)
include_directories(./include/arduino-pico/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include)
include_directories(./include/arduino-pico/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Core/Include)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pwm hardware_dma hardware_pio hardware_exception pico_multicore)
# target_include_directories(${PROJECT_NAME} PUBLIC ./include)

pico_add_extra_outputs(${PROJECT_NAME})
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
