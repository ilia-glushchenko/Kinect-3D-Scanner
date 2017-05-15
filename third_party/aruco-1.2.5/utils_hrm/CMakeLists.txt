INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${GNULIBS_INCLUDE_DIR})
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )

ADD_EXECUTABLE(aruco_hrm_create_dictionary aruco_hrm_create_dictionary.cpp)
ADD_EXECUTABLE(aruco_hrm_test aruco_hrm_test.cpp)
ADD_EXECUTABLE(aruco_hrm_create_board aruco_hrm_create_board.cpp)
ADD_EXECUTABLE(aruco_hrm_test_board aruco_hrm_test_board.cpp)

INSTALL(TARGETS aruco_hrm_create_dictionary aruco_hrm_test aruco_hrm_create_board aruco_hrm_test_board RUNTIME DESTINATION bin)

