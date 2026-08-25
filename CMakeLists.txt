cmake_minimum_required(VERSION 3.14)

project(tiny_phys
    VERSION 0.1.0
    DESCRIPTION "A small 3D rigid body physics engine in C"
    LANGUAGES C)

# Detect whether we are the top-level project. Tests and warnings-as-errors
# should be ON when developing, and OFF when someone FetchContent's us.
if(CMAKE_VERSION VERSION_LESS 3.21)
    get_property(TPHYS_IS_TOP_LEVEL GLOBAL PROPERTY __tphys_top_level SET)
    if(NOT TPHYS_IS_TOP_LEVEL)
        set_property(GLOBAL PROPERTY __tphys_top_level TRUE)
        if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
            set(TPHYS_IS_TOP_LEVEL TRUE)
        else()
            set(TPHYS_IS_TOP_LEVEL FALSE)
        endif()
    endif()
else()
    set(TPHYS_IS_TOP_LEVEL ${PROJECT_IS_TOP_LEVEL})
endif()

option(TPHYS_BUILD_TESTS "Build tiny_phys tests"       ${TPHYS_IS_TOP_LEVEL})
option(TPHYS_WARNINGS_AS_ERRORS "Treat warnings as errors" ${TPHYS_IS_TOP_LEVEL})
option(TPHYS_SANITIZE "Build with ASan + UBSan"        OFF)

# --------------------------------------------------------------- library

add_library(tphys STATIC
    src/tp_world.c
)
add_library(tphys::tphys ALIAS tphys)

target_compile_features(tphys PUBLIC c_std_11)

# BUILD_INTERFACE vs INSTALL_INTERFACE: consumers get include/ from the build
# tree, and <prefix>/include after an install. Without this, `make install`
# produces a broken package.
target_include_directories(tphys
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# libm is a separate library on Linux; on Windows and macOS it is not.
include(CheckLibraryExists)
check_library_exists(m sqrtf "" TPHYS_HAVE_LIBM)
if(TPHYS_HAVE_LIBM)
    target_link_libraries(tphys PUBLIC m)
endif()

if(MSVC)
    target_compile_options(tphys PRIVATE /W4)
    if(TPHYS_WARNINGS_AS_ERRORS)
        target_compile_options(tphys PRIVATE /WX)
    endif()
else()
    target_compile_options(tphys PRIVATE
        -Wall -Wextra -Wpedantic
        -Wshadow -Wconversion -Wdouble-promotion
        -Wstrict-prototypes -Wmissing-prototypes)
    if(TPHYS_WARNINGS_AS_ERRORS)
        target_compile_options(tphys PRIVATE -Werror)
    endif()
endif()

if(TPHYS_SANITIZE AND NOT MSVC)
    target_compile_options(tphys PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer -g)
    target_link_options(tphys PUBLIC -fsanitize=address,undefined)
endif()

# ----------------------------------------------------------------- tests

if(TPHYS_BUILD_TESTS)
    enable_testing()

    foreach(test_name IN ITEMS test_math)
        add_executable(${test_name} tests/${test_name}.c)
        target_link_libraries(${test_name} PRIVATE tphys)
        target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# --------------------------------------------------------------- install

include(GNUInstallDirs)
install(TARGETS tphys EXPORT tphysTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
