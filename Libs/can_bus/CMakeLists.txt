add_library(can_bus STATIC
    can_bus.c
)

target_include_directories(can_bus PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(can_bus PUBLIC
    stm32cubemx
)