# Libs/tmc2209/CMakeLists.txt

add_library(tmc2209 STATIC
    tmc2209.c
)

target_include_directories(tmc2209 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(tmc2209 PUBLIC
    stm32cubemx
)