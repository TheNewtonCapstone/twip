cmake_minimum_required(VERSION 3.10)

# Define the project name
project(twip-jetson)

# Set of options
set(CONTROLLER_TYPE "model" CACHE STRING "Type of controller (Options: model, pid)")

if (NOT (${CONTROLLER_TYPE} STREQUAL "model" OR ${CONTROLLER_TYPE} STREQUAL "pid"))
    message(FATAL_ERROR "Invalid controller type: ${CONTROLLER_TYPE}")
endif()

set(BUILD_ORT OFF CACHE BOOL "Should it build ORT or use the system installation?")
set(GIT_USE_SSH OFF CACHE BOOL "Should Git use SHH instead of HTTPS?")

# Set the C++ standard (optional, depending on your requirement)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (BUILD_ORT)
    # Cmake module to automatically install dependencies
    include(ExternalProject)

    # External project to download and build ONNX Runtime
    set(ORT_TAG v1.19.0)
    set(ORT_DIR ${CMAKE_BINARY_DIR}/onnxruntime)
    set(ORT_INSTALL_DIR ${ORT_DIR}/install)
    set(ORT_BUILD_CONFIG "Release")

    if (GIT_USE_SSH)
        set(ORT_GIT_REPOSITORY "git@github.com:microsoft/onnxruntime.git")
    else()
        set(ORT_GIT_REPOSITORY "https://github.com/microsoft/onnxruntime.git")
    endif()

    message(STATUS "Will download & build ORT (${ORT_TAG}, ${ORT_BUILD_CONFIG}) in ${ORT_DIR}, with ${ORT_GIT_REPOSITORY}.")
    
    ExternalProject_Add(
        onnxruntime
        PREFIX ${ORT_DIR}
        GIT_REPOSITORY ${ORT_GIT_REPOSITORY}
        GIT_TAG ${ORT_TAG}  # Change this to the version needed
        GIT_PROGRESS ON
        GIT_SHALLOW ON
        CONFIGURE_COMMAND "" # Disables CMake configuration
        BUILD_COMMAND bash build.sh --config ${ORT_BUILD_CONFIG} --update --parallel --skip_tests --skip_submodule_sync --cmake_extra_defines CMAKE_INSTALL_PREFIX=${ORT_DIR}/install && cd build/Linux/${ORT_BUILD_CONFIG} && make install
        INSTALL_COMMAND "" # Disables CMake installation
        BUILD_IN_SOURCE ON
    )

    # Ensure that the target directories exist at configure time
    file(MAKE_DIRECTORY ${ORT_INSTALL_DIR}/lib)
    file(MAKE_DIRECTORY ${ORT_INSTALL_DIR}/include/onnxruntime)

    # Tell CMake to use the ONNX Runtime libraries
    add_library(onnxruntime_lib STATIC IMPORTED onnxruntime)
    set_target_properties(onnxruntime_lib PROPERTIES
        IMPORTED_LOCATION ${ORT_INSTALL_DIR}/lib/libonnxruntime_common.a
        INTERFACE_INCLUDE_DIRECTORIES ${ORT_INSTALL_DIR}/include/onnxruntime
    )
else()
    message(STATUS "Will skip building ORT, using system installation.")

    # Tell CMake to use the ONNX Runtime libraries
    add_custom_target(onnxruntime)

    if (NOT EXISTS /usr/local/include/onnxruntime)
        message(FATAL_ERROR "Skipped building ORT, but cannot find system installation.")
    endif()

    add_library(onnxruntime_lib STATIC IMPORTED onnxruntime)
    set_target_properties(onnxruntime_lib PROPERTIES
        IMPORTED_LOCATION /usr/local/include/onnxruntime
        INTERFACE_INCLUDE_DIRECTORIES /usr/local/lib/onnxruntime
    )
endif()

set(MODEL_PROJECT_NAME ${PROJECT_NAME}-model)
set(PID_PROJECT_NAME ${PROJECT_NAME}-pid)

# Add executable
if (${CONTROLLER_TYPE} STREQUAL "model")
    set(SRCS
        src/onnx.cpp
        src/model.cpp
    )

    add_executable(${MODEL_PROJECT_NAME} ${SRCS})

    # Link ORT
    add_dependencies(${MODEL_PROJECT_NAME} onnxruntime)
    target_link_libraries(${MODEL_PROJECT_NAME} onnxruntime_lib)

    set(CONTROLLER_PROJECT_NAME ${MODEL_PROJECT_NAME})
elseif(${CONTROLLER_TYPE} STREQUAL "pid")
    set(SRCS
        src/pid.cpp
    )

    add_executable(${PID_PROJECT_NAME} ${SRCS})

    set(CONTROLLER_PROJECT_NAME ${PID_PROJECT_NAME})
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)
