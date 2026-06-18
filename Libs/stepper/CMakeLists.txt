# Libs/stepper/CMakeLists.txt

add_library(stepper STATIC
    stepper.c
)

target_include_directories(stepper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(stepper PUBLIC
    stm32cubemx
)