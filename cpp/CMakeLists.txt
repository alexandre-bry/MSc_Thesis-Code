cmake_minimum_required(VERSION 3.10)
project(Points2Prints LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/executable)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 1)

# Code
FILE(GLOB SRC_FILES CONFIGURE_DEPENDS src/*.cpp src/**/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# CGAL
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(CGAL REQUIRED COMPONENTS Core)
if ( CGAL_FOUND )
  message(STATUS "CGAL found")
else()
  message(SEND_ERROR "CGAL library is required")
  return()  
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL CGAL::CGAL_Core)

# PDAL
find_package(PDAL CONFIG REQUIRED)
if ( PDAL_FOUND )
  message(STATUS "PDAL found")
else()
  message(SEND_ERROR "PDAL library is required")
  return()  
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE ${PDAL_LIBRARIES})

# # GEOS
# find_package(GEOS CONFIG REQUIRED)
# if (GEOS_FOUND)
#   message(STATUS "GEOS found")
# else()
#   message(SEND_ERROR "GEOS library is required")
#   return()
# endif()
# target_link_libraries(${PROJECT_NAME} PRIVATE GEOS::geos GEOS::geos_c)

# Arrow
find_package(Arrow CONFIG REQUIRED QUIET)
find_package(Parquet CONFIG REQUIRED QUIET)
if ( Arrow_FOUND )
  message(STATUS "Arrow found")
else()
  message(SEND_ERROR "Arrow library is required")
  return()  
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE Arrow::arrow_shared Parquet::parquet_shared)

# GDAL
find_package(GDAL REQUIRED)
if (GDAL_FOUND)
  message(STATUS "GDAL found")
else()
  message(SEND_ERROR "GDAL library is required")
  return()
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE GDAL::GDAL)

# Header-only libraries
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

# install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)