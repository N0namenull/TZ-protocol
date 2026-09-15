cmake_minimum_required(VERSION 3.20)
project(DroneUdp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(drone_core
    src/common/protocol.cpp
    src/drone/model.cpp
    src/gcs/commands.cpp
    src/gcs/reliability.cpp
)
target_include_directories(drone_core PUBLIC src)

if(WIN32)
    add_library(windows_io src/common/udp_socket.cpp src/common/console.cpp
        src/common/terminal_view.cpp src/common/options.cpp)
    target_compile_definitions(windows_io PUBLIC WIN32_LEAN_AND_MEAN NOMINMAX _WIN32_WINNT=0x0601)
    target_link_libraries(windows_io PUBLIC drone_core ws2_32)
    add_executable(DroneSim src/drone/main.cpp src/drone/server.cpp)
    add_executable(GCS src/gcs/main.cpp src/gcs/client.cpp)
    target_link_libraries(DroneSim PRIVATE windows_io)
    target_link_libraries(GCS PRIVATE windows_io)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(CoreTests tests/core_tests.cpp)
    target_link_libraries(CoreTests PRIVATE drone_core)
    add_test(NAME CoreTests COMMAND CoreTests)
    add_executable(GcsTests tests/gcs_tests.cpp)
    target_link_libraries(GcsTests PRIVATE drone_core)
    add_test(NAME GcsTests COMMAND GcsTests)
endif()
