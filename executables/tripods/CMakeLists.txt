file(GLOB EXECUTABLES_TRIPODS_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tripods )
	
if(BUILD_FOR_GTSAM)
	# find_package( OpenCV REQUIRED )
	# include_directories( ${OpenCV_INCLUDE_DIRS} )
	# add_executable( DisplayImage DisplayImage.cpp )
	foreach(f ${EXECUTABLES_TRIPODS_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})

		target_link_libraries(${F_EXEC} ${PROJECT_NAME} gtsam CGAL::CGAL 	)


	endforeach()
endif(BUILD_FOR_GTSAM)