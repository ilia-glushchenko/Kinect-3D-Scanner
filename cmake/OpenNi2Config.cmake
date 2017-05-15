# ===================================================================================
#  OpenNi2 CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(OpenNi2 REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME )
#
#    This file will define the following variables:
#      - OpenNi2_LIBS          : The list of libraries to links against.
#      - OpenNi2_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - OpenNi2_VERSION       : The  version of this PROJECT_NAME build. Example: "1.2.0"
#      - OpenNi2_VERSION_MAJOR : Major version part of VERSION. Example: "1"
#      - OpenNi2_VERSION_MINOR : Minor version part of VERSION. Example: "2"
#      - OpenNi2_VERSION_PATCH : Patch version part of VERSION. Example: "0"
#
# ===================================================================================
INCLUDE_DIRECTORIES("/usr/local/include")
SET(OpenNi2_INCLUDE_DIRS "/usr/local/include" "/usr/local/include/openni2")

LINK_DIRECTORIES("/usr/local/lib")
SET(OpenNi2_LIB_DIR "/usr/local/lib")

SET(OpenNi2_LIBS libOpenNI2.jni.so libOpenNI2.so )

SET(OpenNi2_FOUND 1)
SET(OpenNi2_VERSION        2.2.0.33)
SET(OpenNi2_VERSION_MAJOR  2)
SET(OpenNi2_VERSION_MINOR  2)
SET(OpenNi2_VERSION_PATCH  0.33)
