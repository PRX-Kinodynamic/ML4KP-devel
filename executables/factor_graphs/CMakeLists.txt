file(GLOB EXECUTABLES_FACTOR_GRAPHS_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/factor_graphs )
	

if(BUILD_FOR_GTSAM)
	# find_package( OpenCV REQUIRED )
	# include_directories( ${OpenCV_INCLUDE_DIRS} )
	# add_executable( DisplayImage DisplayImage.cpp )
	foreach(f ${EXECUTABLES_FACTOR_GRAPHS_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})
		target_link_libraries(${F_EXEC} ${PROJECT_NAME}	)
		# target_link_libraries(${F_EXEC} ${OpenCV_LIBS} )
		# if (BUILD_FOR_MUJOCO) 
		# 	target_link_libraries(${F_EXEC} ${PROJECT_NAME}_mjsim ) 
		# endif()
		# target_link_libraries(${F_EXEC} FactorGraphs) 
	endforeach()
endif(BUILD_FOR_GTSAM)