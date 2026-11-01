# Ordinary implementation libraries. Header-only contracts use INTERFACE targets.
function(dart_add_cpp_library target)
    add_library(${target} STATIC ${ARGN})
    target_compile_features(${target} PUBLIC cxx_std_23)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(${target} PUBLIC -fno-exceptions -fno-rtti)
    target_include_directories(${target} PUBLIC
        "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/include/firmware/src")
endfunction()
