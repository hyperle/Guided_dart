# Honor an explicit override; otherwise use CMake's normal CXX/system selection.
if(NOT CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX}
   AND DEFINED ENV{DART_CXX_COMPILER} AND NOT "$ENV{DART_CXX_COMPILER}" STREQUAL "")
    set(CMAKE_CXX_COMPILER "$ENV{DART_CXX_COMPILER}" CACHE FILEPATH "C++ compiler")
endif()
