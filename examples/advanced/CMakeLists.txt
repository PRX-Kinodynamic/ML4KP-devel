
file(GLOB_RECURSE EXAMPLES_ADVANCED_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/advanced )
	
foreach(f ${EXAMPLES_ADVANCED_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC} ${${PROJECT_NAME}_LIBRARIES})
endforeach()
