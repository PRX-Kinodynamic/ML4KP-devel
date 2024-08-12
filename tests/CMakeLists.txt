set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tests )

message("Building tests")
enable_testing()

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
file(GLOB_RECURSE TESTS_FILES
	${PROJECT_SOURCE_DIR}/tests/external/*.cpp
	${PROJECT_SOURCE_DIR}/tests/simulation/*.cpp
	${PROJECT_SOURCE_DIR}/tests/planning/*.cpp
	${PROJECT_SOURCE_DIR}/tests/utilities/*.cpp
	)
if (BUILD_FOR_MUJOCO)
	file(GLOB_RECURSE MJ_TEST_FILES
		${PROJECT_SOURCE_DIR}/tests/mujoco/*.cpp
		)
	list(APPEND TESTS_FILES ${MJ_TEST_FILES})
endif()

if(BUILD_FOR_GTSAM)
	file(GLOB_RECURSE FG_TEST_FILES
		${PROJECT_SOURCE_DIR}/tests/factor_graphs/*.cpp
		)
	list(APPEND TESTS_FILES ${FG_TEST_FILES})
endif()

foreach(f ${TESTS_FILES})
	# get_filename_component(fname ${f} NAME_WE )
	# get_filename_component(f_dir ${f} DIRECTORY )
	# cmake_path(RELATIVE_PATH f_dir)
	file(RELATIVE_PATH test_path_rel ${CMAKE_CURRENT_SOURCE_DIR} ${f})
	get_filename_component(test_dir ${test_path_rel} DIRECTORY )
	get_filename_component(test_path ${test_path_rel} NAME_WE )
	string(REPLACE "/" "_" fname "test/${test_dir}/${test_path}")
	# string(REPLACE "/" "_" fname "test/${f_dir}/${fname}")
	# message("fname ${fname}")
	add_executable(${fname} ${f})
	target_link_libraries(${fname} ${PROJECT_NAME} )
	target_link_libraries(${fname} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
	if (BUILD_FOR_MUJOCO) 
		target_link_libraries(${fname} ${PROJECT_NAME}_mjsim ) 
	endif()
	if(BUILD_FOR_GTSAM)
		target_link_libraries(${fname} gtsam) 
	endif()
	# target_link_libraries(${fname} ${${PROJECT_NAME}_ADDITIONAL_LIBRARIES} ) 
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
