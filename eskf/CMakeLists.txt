cmake_minimum_required(VERSION 3.15)
project(eskf_demo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(eskf_demo main.cpp)
target_link_libraries(eskf_demo PRIVATE Eigen3::Eigen)
if (MSVC)
    target_compile_options(eskf_demo PRIVATE /O2 /W3)
else()
    target_compile_options(eskf_demo PRIVATE -O2 -Wall -Wextra)
endif()
