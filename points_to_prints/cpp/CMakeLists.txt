cmake_minimum_required(VERSION 3.10)
project(Roof_Foot_Extraction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/executable)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 1)

# Code
FILE(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# CGAL
find_package( CGAL QUIET COMPONENTS )
if ( CGAL_FOUND )
  message(STATUS "CGAL found")
else()
  message(SEND_ERROR "CGAL library is required")
  return()  
endif()

# PDAL
find_package(PDAL 2.0.0 REQUIRED CONFIG)
if ( PDAL_FOUND )
  message(STATUS "PDAL found")
else()
  message(SEND_ERROR "PDAL library is required")
  return()  
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE ${PDAL_LIBRARIES})

# GEOS
find_package(GEOS REQUIRED)
if (GEOS_FOUND)
  message(STATUS "GEOS found")
else()
  message(SEND_ERROR "GEOS library is required")
  return()
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE GEOS::geos GEOS::geos_c)

# Arrow
find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)
if ( Arrow_FOUND )
  message(STATUS "Arrow found")
else()
  message(SEND_ERROR "Arrow library is required")
  return()  
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE Arrow::arrow_shared Parquet::parquet_shared)

# Header-only libraries
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

# install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)