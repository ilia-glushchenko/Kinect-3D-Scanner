cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(RoomScanner)


set(CMAKE_CXX_FLAGS "-O2 -march=native -pipe")
set(CMAKE_C_FLAGS   "-O2 -march=native -pipe")


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5SerialPort)

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV 	REQUIRED )
find_package( aruco 	REQUIRED )
include("${CMAKE_CURRENT_LIST_DIR}/cmake/CPUTSDFConfig.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/OpenNi2Config.cmake")

include_directories(
    ./include
    ${PCL_INCLUDE_DIRS}
    ${OpenNi2_INCLUDE_DIRS}
    ${aruco_INCLUDE_DIRS}/aruco
    ${CPUTSDF_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenNi2_LIB_DIR}
)

set(SRC 
    ./include/core/base/imp/scannerbase.cpp
    ./include/core/keypoints/imp/arucokeypointdetector.cpp
    ./include/core/keypoints/imp/keypointsrejection.cpp
    ./include/core/keypoints/imp/surfkeypointdetector.cpp
    ./include/core/reconstruction/imp/reconstructioninterface.cpp
    ./include/core/reconstruction/imp/volumereconstruction.cpp
    ./include/core/registration/imp/elchcorrection.cpp
    ./include/core/registration/imp/icpregistration.cpp
    ./include/core/registration/imp/lumcorrection.cpp
    ./include/core/registration/imp/sacregistration.cpp
    ./include/gui/imp/imagesviewerwidget.cpp
    ./include/gui/imp/pcdvizualizer.cpp
    ./include/gui/imp/scannerwidget.cpp
    ./include/io/imp/calibrationinterface.cpp
    ./include/io/imp/openniinterface.cpp
    ./include/io/imp/pclio.cpp
    ./include/utility/imp/pcdfilters.cpp
    ./include/utility/imp/tools.cpp
    ./main.cpp
    )

set(HEADERS 
    ./include/core/base/scannerbase.h
    ./include/core/base/types.h
    ./include/core/keypoints/arucokeypointdetector.h
    ./include/core/keypoints/keypointsdetector.hpp
    ./include/core/keypoints/keypointsrejection.h
    ./include/core/keypoints/surfkeypointdetector.h
    ./include/core/reconstruction/reconstructioninterface.h
    ./include/core/reconstruction/volumereconstruction.h
    ./include/core/registration/correction.hpp
    ./include/core/registration/edgebalancer.hpp
    ./include/core/registration/edgebasedregistration.hpp
    ./include/core/registration/elchcorrection.h
    ./include/core/registration/errormetric.hpp
    ./include/core/registration/icpregistration.h
    ./include/core/registration/linearbasedregistration.hpp
    ./include/core/registration/linearregistration.hpp
    ./include/core/registration/lumcorrection.h
    ./include/core/registration/middlebasedregistration.hpp
    ./include/core/registration/parallelregistration.hpp
    ./include/core/registration/registration.hpp
    ./include/core/registration/registrationalgorithm.hpp
    ./include/core/registration/sacregistration.h
    ./include/gui/imagesviewerwidget.h
    ./include/gui/pcdvizualizer.h
    ./include/gui/scannerwidget.h
    ./include/io/calibrationinterface.h
    ./include/io/openniinterface.h
    ./include/io/pcdinputiterator.hpp
    ./include/io/pclio.h
    ./include/utility/pcdfilters.h
    ./include/utility/tools.h
)


add_executable (RoomScanner
    ${SRC}
    ${HEADERS}
)

target_link_libraries (RoomScanner
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::SerialPort
    ${CPUTSDF_LIBRARIES}
    ${aruco_LIBS}
    ${OpenNi2_LIBS}
)







