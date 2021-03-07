cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(vcpkg_install)

project(RoomScanner)

include(cmake_config)
include(room_scanner_config)

#VCPKG packages
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Widgets Core CONFIG REQUIRED QUIET)
find_package(OpenCV CONFIG REQUIRED QUIET)
find_package(Qt5SerialPort CONFIG REQUIRED QUIET)
find_package(pcl CONFIG REQUIRED QUIET)

#Submodules
include(cpu_tsdf_config)
include(aruco_config)

#Executable
add_executable (RoomScanner ${ROOM_SCANNER_SRC})
include_directories("src/")
include_directories(SYSTEM
    ${ARUCO_INCLUDE_DIR}
    ${CPU_TSDF_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

target_link_libraries (RoomScanner PRIVATE
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Qt5::Core
    Qt5::Widgets
    Qt5::SerialPort
    cpu_tsdf
    aruco
)

# Create symlink for assets
add_custom_command(
  TARGET RoomScanner
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/default_project
          ${CMAKE_CURRENT_BINARY_DIR}/default_project)

add_custom_command(
    TARGET RoomScanner
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/configs.ini
            ${CMAKE_CURRENT_BINARY_DIR}/configs.ini)

include(grouping)
