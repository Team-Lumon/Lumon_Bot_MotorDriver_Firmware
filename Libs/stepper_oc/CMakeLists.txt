# Libs/stepper_oc/CMakeLists.txt

add_library(stepper_oc STATIC
    stepper_oc.c
)

target_include_directories(stepper_oc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(stepper_oc PUBLIC
    stm32cubemx
)