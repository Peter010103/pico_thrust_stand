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

pico_sdk_init()

SET(DSHOT_FILES ./lib/shoot/shoot.cpp ./lib/tts/tts.cpp ./lib/utils/utils.cpp ./lib/dshot/dshot.cpp)

add_executable(${PROJECT_NAME} src/main.cpp ./include/arduino-pico/cores/rp2040/main.cpp ${DSHOT_FILES})

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
include_directories(./include/arduino-pico/ArduinoCore-API/api)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pwm hardware_dma hardware_pio hardware_exception pico_multicore)

pico_add_extra_outputs(${PROJECT_NAME})
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
