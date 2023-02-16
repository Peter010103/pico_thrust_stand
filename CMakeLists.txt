cmake_minimum_required(VERSION 3.12)

include(lib/extern/pico-sdk/pico_sdk_init.cmake)

# include(pico_sdk_import.cmake)
# include(pico_extras_import.cmake)

project(dshot_test LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

add_compile_options(
  -Wall
  -Wno-format # int != int32_t as far as the compiler is concerned because gcc
              # has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
  -Wno-maybe-uninitialized)

set(uROS_LIB ./lib/micro_ros_raspberrypi_pico_sdk/libmicroros)

pico_sdk_init()
link_directories(${uROS_LIB})

set(DSHOT_FILES ./lib/shoot/shoot.cpp ./lib/tts/tts.cpp ./lib/utils/utils.cpp
                ./lib/dshot/dshot.cpp)

add_executable(${PROJECT_NAME} src/main.cpp ${DSHOT_FILES}
                               ./lib/extern/hx711-pico-c/src/hx711.c
                               ./lib/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c
                               )

include_directories(./lib/shoot)
include_directories(./lib/tts)
include_directories(./lib/utils)
include_directories(./lib/dshot)
include_directories(./lib/extern/hx711-pico-c/include)
include_directories(./include)
include_directories(./lib/micro_ros_raspberrypi_pico_sdk)

target_link_libraries(
  ${PROJECT_NAME}
  pico_stdlib
  pico_platform
  hardware_pwm
  hardware_dma
  hardware_pio
  hardware_exception
  pico_multicore
  microros
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ./lib/micro_ros_raspberrypi_pico_sdk/libmicroros/include
)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(${PROJECT_NAME})
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
