# Libs/debug_helper/CMakeLists.txt

add_library(debug_helper STATIC
    debug_helper.c
)

target_include_directories(debug_helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(debug_helper PUBLIC
    stm32cubemx
)