add_executable(test_algorithm test_algorithm.cpp)
target_link_libraries(test_algorithm ${REQUIRED_LIBRARIES})
add_executable(main main.cpp)
target_link_libraries(main
	${REQUIRED_LIBRARIES})
