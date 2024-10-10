cmake_minimum_required(VERSION 3.10)

# Define the project name
project(Controller)

# Set the C++ standard (optional, depending on your requirement)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# target_compile_features(Controller PUBLIC c_std_99 cxx_std_17)

# setup onnx runtime  dependency 
set(ONNXRUNTIME_INCLUDE_DIR /usr/local/include/onnxruntime)
set(ONNXRUNTIME_BIN_DIR /usr/local/lib/onnxruntime)

# Add include directory (where onnx.hpp is located)
include_directories(
  include 
  "${ONNXRUNTIME_INCLUDE_DIR}/"
)

link_directories("${ONNXRUNTIME_BIN_DIR}/")
set(SOURCES
    src/main.cpp
    src/onnx.cpp
)

# Add executable
add_executable(Controller ${SOURCES})

include_directories(${CMAKE_SOURCE_DIR}/include)

install(TARGETS Controller
      DESTINATION lib/${PROJECT_NAME}
      )


install(DIRECTORY include 
    DESTINATION include/
    )

target_link_libraries(Controller
  onnxruntime
)

# target_link_libraries(O PRIVATE ${ONNX_LIBRARIES})

