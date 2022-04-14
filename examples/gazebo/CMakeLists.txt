if(BUILD_FOR_GAZEBO)
	# find_package(gazebo REQUIRED)

	file(GLOB_RECURSE EXAMPLES_PRXGZ_FILES *.cpp)

	set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/gazebo )

	foreach(f ${EXAMPLES_PRXGZ_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})
		target_link_libraries(${F_EXEC} PUBLIC
			${PROJECT_NAME}_external 
			${PROJECT_NAME}_utilities 
			${PROJECT_NAME}_simulation 
			${PROJECT_NAME}_planning 	
			${PROJECT_NAME}_visualization 
			)
		target_link_libraries(${F_EXEC} PUBLIC	${PROJECT_NAME}_gz)
		target_link_libraries(${F_EXEC} PUBLIC  ${PRX_GZ_LIBRARIES})
		target_link_libraries(${F_EXEC} PRIVATE ${GAZEBO_LIBRARIES})

		if (BUILD_FOR_TORCH)
			target_link_libraries(${F_EXEC} PUBLIC ${TORCH_LIBRARIES})
		endif(BUILD_FOR_TORCH)
	endforeach()

endif(BUILD_FOR_GAZEBO)