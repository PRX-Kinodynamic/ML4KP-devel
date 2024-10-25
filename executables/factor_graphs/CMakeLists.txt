file(GLOB EXECUTABLES_FACTOR_GRAPHS_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/factor_graphs )
	
find_package(CGAL REQUIRED)
find_package(nlohmann_json QUIET)
	# find_package(GTSAM REQUIRED)
if( nlohmann_json_FOUND )
	message(STATUS "Json library nlohmann_json Found ")
  # COMPILE_DEFINITIONS(JSON)
	set(COMPILE_DEFINITIONS "JSON")
  # add_compile_definitions(-DJSON)
endif()
if(BUILD_FOR_GTSAM)
	# find_package( OpenCV REQUIRED )
	# include_directories( ${OpenCV_INCLUDE_DIRS} )
	# add_executable( DisplayImage DisplayImage.cpp )
	foreach(f ${EXECUTABLES_FACTOR_GRAPHS_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})

		target_link_libraries(${F_EXEC} ${PROJECT_NAME} gtsam CGAL::CGAL 	)

		if( nlohmann_json_FOUND )
			target_link_libraries(${F_EXEC}  nlohmann_json::nlohmann_json)
		endif()

	endforeach()
endif(BUILD_FOR_GTSAM)