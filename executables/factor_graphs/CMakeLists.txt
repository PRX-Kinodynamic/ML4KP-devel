file(GLOB EXECUTABLES_FACTOR_GRAPHS_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/factor_graphs )
	

if(BUILD_FOR_GTDYNAMICS)
	# find_package(gtdynamics REQUIRED)
	
	foreach(f ${EXECUTABLES_FACTOR_GRAPHS_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})
		target_link_libraries(${F_EXEC}
			${PROJECT_NAME}_external 
			${PROJECT_NAME}_utilities 
			${PROJECT_NAME}_simulation 
			${PROJECT_NAME}_planning 	
			${PROJECT_NAME}_visualization 
			${PROJECT_NAME}_gtdynamics
			)
		if(BUILD_FOR_BULLET)
			target_link_libraries(${F_EXEC} ${PROJECT_NAME}_bullet_sim )
		endif(BUILD_FOR_BULLET)
		if (BUILD_FOR_TORCH)
			target_link_libraries(${F_EXEC} ${TORCH_LIBRARIES})
		endif(BUILD_FOR_TORCH)
	endforeach()
endif(BUILD_FOR_GTDYNAMICS)