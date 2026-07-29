# Libs/tmc_stall/CMakeLists.txt

add_library(tmc_stall STATIC
    tmc_stall.c
)

target_include_directories(tmc_stall PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(tmc_stall PUBLIC
    stm32cubemx
    tmc2209
    motor_controller
)
