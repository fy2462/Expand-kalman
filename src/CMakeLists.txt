add_executable(${PROJECT_NAME} main.cpp
        filter.cpp
        ekf_core.cpp
        common/command_args.cpp
        common/tools.cpp)
target_link_libraries(${PROJECT_NAME})