set(ROOM_SCANNER_NAME
    "RoomScanner"
    CACHE STRING "Room scanner project name.")

if(NOT DEFINED ROOM_SCANNER_ROOT)
  set(ROOM_SCANNER_ROOT
      "${CMAKE_CURRENT_SOURCE_DIR}"
      CACHE STRING "Room scanner root directory.")
endif()

file(GLOB_RECURSE ROOM_SCANNER_SRC "src/*.hpp" "src/*.h" "src/*.cpp")
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ROOM_SCANNER_SRC})

if(MSVC)
  add_compile_options(/MP /bigobj)
endif()
