project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)
add_definitions(-DVISUAL_DEBUG)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/helpers.cpp
            src/cost_functions.cpp
            src/ptg.cpp
            src/main.cpp)
            #src/main_test.cpp)

find_package(PythonLibs 2.7)
find_package(OpenCV REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS
                      ${PYTHON_LIBRARIES}
                      ${OpenCV_LIBS})
