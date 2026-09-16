add_executable(bridge_test ${CMAKE_SOURCE_DIR}/example/bridge_test.cpp)
add_executable(record_joint ${CMAKE_SOURCE_DIR}/example/record_joint.cpp)
add_executable(rpc_test ${CMAKE_SOURCE_DIR}/example/rpc_test.cpp)
add_executable(switch_upper ${CMAKE_SOURCE_DIR}/example/switch_upper.cpp)
add_executable(keyboard_control ${CMAKE_SOURCE_DIR}/example/keyboard_control.cpp)

set(EXAMPLE_TARGETS
    bridge_test
    record_joint
    rpc_test
    switch_upper
    keyboard_control
    )
foreach(target ${EXAMPLE_TARGETS})
    target_link_libraries(${target} PRIVATE ${SDK_LIBS_DIR})
    target_include_directories(${target} PRIVATE ${SDK_INCLUDE_DIR})
endforeach()

