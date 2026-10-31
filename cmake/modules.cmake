# CMake scans every module and consumer; unity builds must never concatenate them.
set(CMAKE_CXX_SCAN_FOR_MODULES ON)
if(NOT CMAKE_GENERATOR MATCHES "^Ninja")
    message(FATAL_ERROR "Use the Ninja generator for named modules (-G Ninja)")
endif()
execute_process(COMMAND "${CMAKE_MAKE_PROGRAM}" --version
    OUTPUT_VARIABLE _dart_ninja_version OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_dart_ninja_version VERSION_LESS 1.11)
    message(FATAL_ERROR "Named modules need Ninja >= 1.11")
endif()
if(CMAKE_UNITY_BUILD)
    message(FATAL_ERROR "C++ modules require separate translation units; disable CMAKE_UNITY_BUILD")
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14)
        message(FATAL_ERROR "Named modules need GCC >= 14, or Clang >= 19 with clang-scan-deps; select a supported CMAKE_CXX_COMPILER")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Cross compiler prefixes can make CMake search for a prefixed scanner.
    # clang-scan-deps is always a host executable beside the Clang installation.
    if(NOT EXISTS "${CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS}")
        get_filename_component(_dart_clang_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
        string(REGEX MATCH "^[0-9]+" _dart_clang_major "${CMAKE_CXX_COMPILER_VERSION}")
        find_program(CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS
            NAMES "clang-scan-deps-${_dart_clang_major}" clang-scan-deps
            HINTS "${_dart_clang_bin}" NO_CMAKE_FIND_ROOT_PATH)
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19 OR NOT EXISTS "${CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS}")
        message(FATAL_ERROR "Named modules need Clang >= 19 and the matching clang-scan-deps (clang-tools package)")
    endif()
else()
    message(FATAL_ERROR "Use GCC >= 14 or Clang >= 19 for the named-module build")
endif()

function(dart_add_modules target)
    add_library(${target} STATIC)
    target_sources(${target} PUBLIC FILE_SET CXX_MODULES BASE_DIRS "${PROJECT_SOURCE_DIR}" FILES ${ARGN})
    target_compile_features(${target} PUBLIC cxx_std_23)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(${target} PUBLIC -fno-exceptions -fno-rtti)
    target_include_directories(${target} PUBLIC
        "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/include/firmware/src")
endfunction()
