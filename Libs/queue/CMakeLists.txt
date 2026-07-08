add_library(queue STATIC
    queue.c
)

target_include_directories(queue PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(queue PUBLIC
    stm32cubemx
)
