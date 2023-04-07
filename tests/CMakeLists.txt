set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tests )

message("Building tests")
enable_testing()

#ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
file(GLOB_RECURSE TESTS_FILES
	${PROJECT_SOURCE_DIR}/tests/simulation/*.cpp
	${PROJECT_SOURCE_DIR}/tests/planning/*.cpp
	${PROJECT_SOURCE_DIR}/tests/utilities/*.cpp
	)

foreach(f ${TESTS_FILES})
	GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
	add_executable(${fname} ${f})
	target_link_libraries(${fname} ${PROJECT_NAME}_external )
	target_link_libraries(${fname} ${PROJECT_NAME}_utilities )
	target_link_libraries(${fname} ${PROJECT_NAME}_visualization )
	target_link_libraries(${fname} ${PROJECT_NAME}_simulation )
	target_link_libraries(${fname} ${PROJECT_NAME}_planning )
	if(BUILD_FOR_BULLET)
		target_link_libraries(${fname} ${PROJECT_NAME}_bullet_sim )
	endif()
	target_link_libraries(${fname} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
	add_test(NAME ${fname}_test COMMAND ${fname})
	# add_test(NAME ${fname}_test COMMAND ${fname} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endforeach()

