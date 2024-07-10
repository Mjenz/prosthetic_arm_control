# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(threading_test)

# Add the include directory where util.h is located
target_include_directories(app PRIVATE include/)

file(GLOB APP_SOURCES src/*.c)
# Add the source files to your project
target_sources(app PRIVATE ${APP_SOURCES})

