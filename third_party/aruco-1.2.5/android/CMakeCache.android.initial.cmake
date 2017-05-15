########################
# Initial cache settings for opencv on android
# run cmake with:
# cmake -C 
########################

#Build shared libraries (.dll/.so CACHE BOOL "" ) instead of static ones (.lib/.a CACHE BOOL "" )
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" )

#Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" )

#
set(ANDROID_CREATION ON CACHE INTERNAL "" FORCE )

#Enable SSE instructions
SET( USE_SSE OFF CACHE INTERNAL "" FORCE )

#Enable SSE2 instructions
SET( USE_SSE2 OFF CACHE INTERNAL "" FORCE )

#Enable SSE3 instructions
SET( USE_SSE3 OFF CACHE INTERNAL "" FORCE )

#Enable SSE3 instructions
SET( GL_FOUND OFF CACHE INTERNAL "" FORCE )

#Set output folder to ${CMAKE_BINARY_DIR}
set( LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_BINARY_DIR} CACHE PATH "root for library output, set this to change where android libs are compiled to" )
