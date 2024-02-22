cmake_minimum_required(VERSION 3.28)

find_program(AVR_CC avr-gcc REQUIRED)
find_program(AVR_CXX avr-g++ REQUIRED)
find_program(AVR_OBJCOPY avr-objcopy REQUIRED)
find_program(AVR_SIZE avr-size REQUIRED)
find_program(AVR_OBJDUMP avr-objdump REQUIRED)

set(MCU attiny88)
set(F_CPU 16000000)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER ${AVR_CC})
set(CMAKE_CXX_COMPILER ${AVR_CXX})

project(NixieClock)

add_compile_options(
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -funsigned-char
    -funsigned-bitfields
    -ffunction-sections
    -fdata-sections
    -fpack-struct
    -fshort-enums
    -Ofast
    -Wall
    -Werror
    -pedantic
)

add_link_options(
    -mmcu=${MCU}
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--start-group
    -Wl,-lm
    -Wl,--end-group
    -Wl,--gc-sections
)

add_executable(${PROJECT_NAME}.elf src/indication.c src/init.c src/main.c)
add_subdirectory(lib)
target_include_directories(${PROJECT_NAME}.elf PUBLIC inc)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
COMMAND ${AVR_OBJCOPY} -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.eep || exit 0)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
COMMAND ${AVR_OBJDUMP} -h -S ${PROJECT_NAME}.elf > ${PROJECT_NAME}.lss)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
COMMAND ${AVR_OBJCOPY} -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures ${PROJECT_NAME}.elf ${PROJECT_NAME}.srec)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
COMMAND ${AVR_SIZE} ${PROJECT_NAME}.elf)