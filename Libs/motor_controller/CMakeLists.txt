# Libs/motor_controller/CMakeLists.txt

add_library(motor_controller STATIC
    motor_controller.c
)

target_include_directories(motor_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(motor_controller PUBLIC
    stm32cubemx
    stepper_oc
    as5600
)