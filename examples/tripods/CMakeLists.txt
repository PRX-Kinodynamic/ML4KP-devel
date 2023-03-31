
file(GLOB_RECURSE EXAMPLES_TRIPODS_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tripods )
	
foreach(f ${EXAMPLES_TRIPODS_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC}
		${PROJECT_NAME}_external 
		${PROJECT_NAME}_utilities 
		${PROJECT_NAME}_simulation 
		${PROJECT_NAME}_planning 	
		${PROJECT_NAME}_visualization 
		${PROJECT_NAME}_visualization 
		# libqhullstatic_r
		# ${CMAKE_BINARY_DIR}/external/qhull/build/libqhullcpp.a
		# ${CMAKE_BINARY_DIR}/external/qhull/build/libqhullstatic_r.a
		)
	if (BUILD_FOR_TORCH)
		target_link_libraries(${F_EXEC} ${TORCH_LIBRARIES})
	endif(BUILD_FOR_TORCH)
endforeach()
