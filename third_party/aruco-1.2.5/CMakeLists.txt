# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(aruco)
set(PROJECT_VERSION "1.2.5")
string(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
list(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
list(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
list(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")


#------------------------------------------------------
# Build type
#------------------------------------------------------

IF(NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE "Debug" )
ENDIF()

#------------------------------------------------------
# Lib Names and Dirs
#------------------------------------------------------

if(WIN32)
    # Postfix of DLLs:
    SET(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")
    SET(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION)
#    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/ /usr/lib/cmake)
endif()


OPTION(BUILD_SHARED_LIBS 	"Set to OFF to build static libraries" ON)

# ----------------------------------------------------------------------------
#   Find Dependencies
# ----------------------------------------------------------------------------

find_package(OpenCV CONFIG REQUIRED QUIET)
set(REQUIRED_LIBRARIES
    opencv_ml
    opencv_ts
    opencv_gpu
    opencv_ocl)

IF(EXISTS ${GLUT_PATH})
    INCLUDE_DIRECTORIES(${GLUT_PATH}/include ${OpenCV_INCLUDE_DIRS})
    SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${GLUT_PATH}/lib)
ENDIF()

##LOOK FOR OPENGL AND GLUT
#FIND OPENGL LIBRARY. In Android, there is no need since it is included
IF(NOT ANDROID_CREATION)
  find_package(OpenGL)
  find_package(GLUT)#standard package

  MESSAGE(STATUS "GLUT_FOUND=${GLUT_FOUND} OPENGL_gl_LIBRARY=${OPENGL_gl_LIBRARY} GLUT_HEADER=${GLUT_HEADER}")
  IF(NOT GLUT_FOUND) #else, freeglut
    FIND_LIBRARY(GLUT_glut_LIBRARY     NAMES freeglut)
    MESSAGE(STATUS "GLUT_glut_LIBRARY=${GLUT_glut_LIBRARY}")
  ENDIF()
  IF ( (NOT GLUT_glut_LIBRARY AND NOT GLUT_FOUND) OR NOT OPENGL_gl_LIBRARY)
    set(GL_FOUND "NO")
  ELSE()
    set(GL_FOUND "YES")
    set (OPENGL_LIBS  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})
  ENDIF()
ENDIF()
#Omp

if(WIN32)

ELSE()
  add_definitions(-DUSE_OMP -fopenmp)
  set (REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} -lgomp)
ENDIF()

# ----------------------------------------------------------------------------
#   Program Optimization and debug (Extracted from OpenCV)
# ----------------------------------------------------------------------------
set(WARNINGS_ARE_ERRORS 		OFF CACHE BOOL "Treat warnings as errors")
set(WHOLE_PROGRAM_OPTIMIZATION 	OFF CACHE BOOL "Flags for whole program optimization.")

set(EXTRA_C_FLAGS "")
set(EXTRA_C_FLAGS_RELEASE "")
set(EXTRA_C_FLAGS_DEBUG "")
set(EXTRA_EXE_LINKER_FLAGS "")
set(EXTRA_EXE_LINKER_FLAGS_RELEASE "")
set(EXTRA_EXE_LINKER_FLAGS_DEBUG "")

IF(CMAKE_COMPILER_IS_GNUCXX OR MINGW)
    set(ENABLE_PROFILING 		OFF CACHE BOOL "Enable profiling in the GCC compiler (Add flags: -g -pg)")
    set(USE_OMIT_FRAME_POINTER 	ON CACHE BOOL "Enable -fomit-frame-pointer for GCC")
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*) # We can use only -O2 because the -O3 causes gcc crash
        set(USE_O2 ON CACHE BOOL "Enable -O2 for GCC")
        set(USE_FAST_MATH ON CACHE BOOL "Enable -ffast-math for GCC")
    endif()
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES powerpc*)
        set(USE_O3 ON CACHE BOOL "Enable -O3 for GCC")
        set(USE_POWERPC ON CACHE BOOL "Enable PowerPC for GCC")
    endif ()
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64*)
        set(USE_O3 ON CACHE BOOL "Enable -O3 for GCC")
        set(USE_FAST_MATH ON CACHE BOOL "Enable -ffast-math for GCC")
        set(USE_MMX ON CACHE BOOL "Enable MMX for GCC")
        set(USE_SSE ON CACHE BOOL "Enable SSE for GCC")
        set(USE_SSE2 ON CACHE BOOL "Enable SSE2 for GCC")
        set(USE_SSE3 ON CACHE BOOL "Enable SSE3 for GCC")
    endif()
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES i686* OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86)
        set(USE_O3 ON CACHE BOOL "Enable -O3 for GCC")
        set(USE_FAST_MATH ON CACHE BOOL "Enable -ffast-math for GCC")
        set(USE_MMX ON CACHE BOOL "Enable MMX for GCC")
        set(USE_SSE OFF CACHE BOOL "Enable SSE for GCC")
        set(USE_SSE2 OFF CACHE BOOL "Enable SSE2 for GCC")
        set(USE_SSE3 OFF CACHE BOOL "Enable SSE3 for GCC")
    endif ()

    set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wall")

    if(WARNINGS_ARE_ERRORS)
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Werror")
    endif()

    # The -Wno-long-long is required in 64bit systems when including sytem headers.
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64*)
		set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-long-long")
    endif()

    # Whole program optimization
    if(WHOLE_PROGRAM_OPTIMIZATION)
        set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -fwhole-program --combine")
    endif()

    # Other optimizations
    if(USE_OMIT_FRAME_POINTER)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -fomit-frame-pointer")
    endif()
    if(USE_O2)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -O2")
    endif()
    if(USE_O3)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -O3")
    endif()
    if(USE_FAST_MATH)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -ffast-math")
    endif()
    if(USE_POWERPC)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -mcpu=G3 -mtune=G5")
    endif()
    if(USE_MMX)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -mmmx")
    endif()
    if(USE_SSE)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -msse")
    endif()
    if(USE_SSE2)
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -msse2")
    endif()
    if(USE_SSE3 AND NOT MINGW) # SSE3 should be disabled under MingW because it generates compiler errors
       set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -msse3")
    endif()

    if(ENABLE_PROFILING)
        set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -pg -g")
    else()
        if(NOT APPLE)
            set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -ffunction-sections")
        endif()
    endif()

    # Parallel mode
    if(ENABLE_OPENMP)
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS}  -fopenmp")
        set(LINKER_LIBS ${LINKER_LIBS} gomp)
    endif()
    IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l) # In ARM_COrtex8 with neon, enalble vectorized operations
	SET(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=softfp -ftree-vectorize ")
    ENDIF()


    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -DNDEBUG")
    set(EXTRA_C_FLAGS_DEBUG "-g3 -O0 -DDEBUG -D_DEBUG -W -Wextra -Wno-return-type")

    MESSAGE( STATUS "-------------------------------------------------------------------------------" )
    message( STATUS "GNU COMPILER")
    MESSAGE( STATUS "-------------------------------------------------------------------------------" )

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE}")
    SET(CMAKE_CXX_FLAGS_DEBUG "${EXTRA_C_FLAGS_DEBUG}")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
    SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")
ENDIF	()

#------------------------------------------------
# DIRS
#------------------------------------------------
ADD_SUBDIRECTORY(src)
