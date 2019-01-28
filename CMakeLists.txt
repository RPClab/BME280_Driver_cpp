########################################################
# Top-level CMakeLists.txt to build BME280-Driver Libraries
# @author LAGARDE Francois
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
########################################################

# ----- Project name and version -----
project(BME280-Driver)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/bin CACHE PATH "..." FORCE)
endif()

set( ${PROJECT_NAME}_VERSION_MAJOR 3 )
set( ${PROJECT_NAME}_VERSION_MINOR 3 )
set( ${PROJECT_NAME}_VERSION_PATCH 4 )

# set the C++ standard to C++ 17
set(CMAKE_CXX_STANDARD 17)

##BME280-Driver Sources
set(BME280-Driver ${PROJECT_SOURCE_DIR}/bme280.cpp ${PROJECT_SOURCE_DIR}/data.cpp ${PROJECT_SOURCE_DIR}/calib.cpp)
file(GLOB BME280_DRIVER_PUBLIC_HEADERS ${PROJECT_SOURCE_DIR}/*.hpp)
include_directories(${PROJECT_SOURCE_DIR})

add_library(bme280 STATIC ${BME280-Driver})

# install configuration
install(FILES ${BME280_DRIVER_PUBLIC_HEADERS} DESTINATION include/bme280)
install(TARGETS bme280 ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/bin RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib)

add_executable(Test ${PROJECT_SOURCE_DIR}/examples/linux_userspace.cpp)
target_link_libraries(Test bme280)
install(TARGETS bme280 ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/bin RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib)
