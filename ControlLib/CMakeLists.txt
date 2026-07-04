cmake_minimum_required(VERSION 3.20)
project(Manager)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-D__DEBUG__)

include_directories(include 3rdparty/include)

file(GLOB_RECURSE SOURCE demo/demo1.cpp ./src/*.cpp ./src/*.cc)

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/libserial.a)
    execute_process(
            COMMAND make
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/src-serial/
    )
    execute_process(
            COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/src-serial/build/libserial.a ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif ()

add_executable(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/libserial.a)