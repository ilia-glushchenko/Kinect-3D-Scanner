# - Config file for the FooBar package
# It defines the following variables
#  CPUTSDF_INCLUDE_DIRS - include directories for CPUTSDF
#  CPUTSDF_LIBRARIES    - libraries to link against
#  TSDF2MESH_CPUTSDF_EXECUTABLE   - the TSDF2MESH executable
#  INTEGRATE_CPUTSDF_EXECUTABLE   - the integrate executable

# Find and Set the library
FIND_LIBRARY(CPUTSDF_LIBRARY cpu_tsdf
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
)
SET(CPUTSDF_LIBRARIES ${CPUTSDF_LIBRARY})

# Find and set the include dirs
SET(CPUTSDF_INCLUDE_DIR "/usr/local/include/cpu_tsdf")

SET(CPUTSDF_INCLUDE_DIRS ${CPUTSDF_INCLUDE_DIR})

# Find and set the executables
SET(TSDF2MESH_CPUTSDF_EXECUTABLE "/usr/local/bin/tsdf2mesh")
SET(INTEGRATE_CPUTSDF_EXECUTABLE "/usr/local/bin/integrate")
