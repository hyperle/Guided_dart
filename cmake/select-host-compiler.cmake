# Honor explicit compiler choices; otherwise prefer a module-capable installation.
if(NOT CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
    if(DEFINED ENV{DART_CXX_COMPILER} AND NOT "$ENV{DART_CXX_COMPILER}" STREQUAL "")
        set(CMAKE_CXX_COMPILER "$ENV{DART_CXX_COMPILER}" CACHE FILEPATH "C++ module compiler")
    else()
        find_program(_dart_host_cxx NAMES g++-15 g++-14 clang++-21 clang++-20 clang++-19
            HINTS "${CMAKE_CURRENT_LIST_DIR}/../.cache/module-toolchain/bin")
        if(_dart_host_cxx)
            set(CMAKE_CXX_COMPILER "${_dart_host_cxx}" CACHE FILEPATH "C++ module compiler")
        endif()
    endif()
endif()
