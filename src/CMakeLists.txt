cmake_minimum_required(VERSION 3.12)
project(simulator)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../extern/box2d/build/lib/cmake/box2d")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../extern/json/build")
set(CMAKE_CXX_STANDARD 17)
set(box2d_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../extern/box2d/build/lib/cmake/box2d")
set(nlohmann_json_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../extern/json/build")

find_package(pybind11 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(box2d REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../extern/box2d/build/bin")

pybind11_add_module(simulator fcv1_simulator.cpp)

set_target_properties(simulator PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_RELEASE       "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE       "${CMAKE_BINARY_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE       "${CMAKE_BINARY_DIR}"
)

if (WIN32)
    set_target_properties(simulator PROPERTIES
        PREFIX ""
        OUTPUT_NAME "simulator"
        SUFFIX ".pyd"
    )
else()
    set_target_properties(simulator PROPERTIES
        PREFIX ""
        OUTPUT_NAME "simulator"
        SUFFIX ".so"
    )
endif()


target_include_directories(simulator PRIVATE "${CMAKE_SOURCE_DIR}/../extern/Box2D/include")
target_link_libraries(simulator PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(simulator PRIVATE pybind11::module)