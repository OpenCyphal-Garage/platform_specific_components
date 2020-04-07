#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#

cmake_minimum_required(VERSION 3.6.0)

project(nxp_s32k ASM CXX C)


configure_file(libuavcan.txt.in libuavcan-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libuavcan-download )
if(result)
    message(FATAL_ERROR "CMake step for libuavcan failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libuavcan-download )
if(result)
    message(FATAL_ERROR "Build step for libuavcan failed: ${result}")
endif()


include_directories(
    .
    ../S32K/module
    ${CMAKE_CURRENT_BINARY_DIR}/libuavcan-src/libuavcan/include
)


set(CMAKE_CXX_FLAGS
   "\
    -std=c++11 \
    -fno-rtti \
    -fno-exceptions \
    -Wsign-conversion \
    -Wsign-promo \
    -Wold-style-cast \
    -Wzero-as-null-pointer-constant \
    -Wnon-virtual-dtor \
    -Woverloaded-virtual \
    "
)

set(CMAKE_C_FLAGS
   "\
    -pedantic \
    -Wall \
    -Wextra \
    -Werror \
    -Wfloat-equal \
    -Wconversion \
    -Wabi \
    -Wunused-parameter \
    -Wunused-variable \
    -Wunused-value \
    -Wcast-align \
    -Wmissing-declarations \
    -Wmissing-field-initializers \
    -Wdouble-promotion \
    -Wswitch-enum \
    -Wtype-limits \
    -Wno-error=array-bounds \
    -fprofile-arcs \
    -ftest-coverage \
    --coverage \
    -O0 \
    -DDEBUG \
    -ggdb \
    "
)


add_executable(test_sanity test_sanity.cpp ../S32K/module/canfd.cpp)
add_test(NAME run_test_sanity COMMAND test_sanity)
