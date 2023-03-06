file(GLOB EXECUTABLES_TENSEGRITY_FILES *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tensegrity )
	

if(BUILD_FOR_FACTOR_GRAPHS)
	find_package( OpenCV REQUIRED )
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	# add_executable( DisplayImage DisplayImage.cpp )
	foreach(f ${EXECUTABLES_TENSEGRITY_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})
		target_link_libraries(${F_EXEC}
			${PROJECT_NAME}_external 
			${PROJECT_NAME}_utilities 
			${PROJECT_NAME}_simulation 
			${PROJECT_NAME}_planning 	
			${PROJECT_NAME}_visualization 
			${PROJECT_NAME}_factor_graphs
			)
		target_link_libraries( ${F_EXEC} ${OpenCV_LIBS} )
		if(BUILD_FOR_BULLET)
			target_link_libraries(${F_EXEC} ${PROJECT_NAME}_bullet_sim )
		endif(BUILD_FOR_BULLET)
		if (BUILD_FOR_TORCH)
			target_link_libraries(${F_EXEC} ${TORCH_LIBRARIES})
		endif(BUILD_FOR_TORCH)
		if(BUILD_FOR_MUJOCO)
			target_link_libraries(${F_EXEC} ${PROJECT_NAME}_mjsim)
		endif(BUILD_FOR_MUJOCO)
	endforeach()
endif(BUILD_FOR_FACTOR_GRAPHS)