
file(GLOB_RECURSE EXAMPLES_BASIC_FILES *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/basic )
	
foreach(f ${EXAMPLES_BASIC_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC} ${PROJECT_NAME})
endforeach()
