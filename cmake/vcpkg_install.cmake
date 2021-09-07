if(NOT DEFINED ENV{VCPKG_ROOT})
  message(FATAL_ERROR "Can't find enviroment variable VCPKG_ROOT")
endif()

set(VCPKG_TARGET_TRIPLET x64-windows)
set(VCPKG_EXE_NAME $ENV{VCPKG_ROOT}\\vcpkg.exe)
set(VCPKG_INCLUDE
    $ENV{VCPKG_ROOT}\\installed\\${VCPKG_TARGET_TRIPLET}\\include)
set(VCPKG_LIB
    $ENV{VCPKG_ROOT}\\installed\\${VCPKG_TARGET_TRIPLET}\\lib)

set(VCPKG_PACKAGES
    qt5[serialport]
    opencv2
    pcl[opengl,openni2,qt,vtk])

set(CMAKE_TOOLCHAIN_FILE
    "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "")

message("-- VCPKG_ROOT is $ENV{VCPKG_ROOT}")
message("-- CMAKE_TOOLCHAIN_FILE is ${CMAKE_TOOLCHAIN_FILE}")
message("-- VCPKG_TARGET_TRIPLET is ${VCPKG_TARGET_TRIPLET}")
message("-- VCPKG_INCLUDE is ${VCPKG_INCLUDE}")
message("-- VCPKG_PACKAGES are ${VCPKG_PACKAGES}")
message(
  "-- Downloading vcpkg dependencies ${VCPKG_EXE_NAME} install ${VCPKG_PACKAGES} --triplet ${VCPKG_TARGET_TRIPLET}"
)

execute_process(COMMAND ${VCPKG_EXE_NAME} install --recurse ${VCPKG_PACKAGES} --triplet
                        ${VCPKG_TARGET_TRIPLET})
execute_process(COMMAND ${VCPKG_EXE_NAME} upgrade --no-dry-run)
