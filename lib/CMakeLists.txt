add_subdirectory(PID)
add_subdirectory(RTC)
add_subdirectory(TWI)

target_link_libraries(${PROJECT_NAME}.elf PUBLIC PID RTC TWI)