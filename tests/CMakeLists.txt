set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tests )

message("Building tests")
enable_testing()

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
file(GLOB_RECURSE TESTS_FILES
	${PROJECT_SOURCE_DIR}/tests/simulation/*.cpp
	${PROJECT_SOURCE_DIR}/tests/planning/*.cpp
	${PROJECT_SOURCE_DIR}/tests/utilities/*.cpp
	)
if (BUILD_FOR_MUJOCO)
	file(GLOB_RECURSE MUJOCO_TEST_FILES
		${PROJECT_SOURCE_DIR}/tests/mujoco/*.cpp
		)
	list(APPEND TESTS_FILES ${MUJOCO_TEST_FILES})
endif()

if (BUILD_FOR_FACTOR_GRAPHS)
	file(GLOB_RECURSE FACTOR_GRAPH_TEST_FILES
		${PROJECT_SOURCE_DIR}/tests/factor_graphs/*.cpp
		)
	list(APPEND TESTS_FILES ${FACTOR_GRAPH_TEST_FILES})
endif()

foreach(f ${TESTS_FILES})
	GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
	GET_FILENAME_COMPONENT(f_dir ${f} DIRECTORY )
	cmake_path(RELATIVE_PATH f_dir)
	string(REPLACE "/" "_" fname "test/${f_dir}/${fname}")
	# message("fname ${fname}")
	add_executable(${fname} ${f})
	target_link_libraries(${fname} ${PROJECT_NAME}_external )
	target_link_libraries(${fname} ${PROJECT_NAME}_utilities )
	target_link_libraries(${fname} ${PROJECT_NAME}_visualization )
	target_link_libraries(${fname} ${PROJECT_NAME}_simulation )
	target_link_libraries(${fname} ${PROJECT_NAME}_planning )
	if(BUILD_FOR_BULLET)
		target_link_libraries(${fname} ${PROJECT_NAME}_bullet_sim )
	endif()
	if (BUILD_FOR_MUJOCO)
		target_link_libraries(${fname} ${PROJECT_NAME}_mjsim )
	endif()
	if (BUILD_FOR_FACTOR_GRAPHS)
		target_link_libraries(${fname} ${PROJECT_NAME}_factor_graphs )
	endif()
	target_link_libraries(${fname} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
	add_test(NAME "${fname}" COMMAND "${fname}" )
	# add_test(NAME ${fname}_test COMMAND ${fname} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endforeach()

if (BUILD_TESTS AND BUILD_FOR_PYTHON)
	find_package (Python COMPONENTS Interpreter)
    add_test (NAME python-tests
      COMMAND ${Python_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR}
      WORKING_DIRECTORY .
    )
endif()
