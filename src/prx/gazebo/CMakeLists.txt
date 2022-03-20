## gazebo submodule of DirtMP



if(BUILD_FOR_GAZEBO)
	include(gazebo.cmake)

	find_package(Protobuf REQUIRED)
	find_package(gazebo REQUIRED)

	set( PRX_GZ_LIBRARIES "" CACHE STRING "PRX + GZ Libraries to link to")
	list(APPEND PRX_GZ_LIBRARIES ${GAZEBO_LIBRARIES})
	# find_ign_lib(ignition-common 3 1)
	# find_package(ignition-common3 QUIET)
	# find_package(ignition-msgs4 QUIET)
	# find_package(ignition-msgs5 QUIET)
	# find_package(ignition-transport7 QUIET)
	# find_package(ignition-transport8 QUIET)

	file(GLOB_RECURSE SRC_PRX_LIB_GZ_SIM  
		gz_sim.cpp
	)

	add_library(${PROJECT_NAME}_gz ${SRC_PRX_LIB_GZ_SIM})

	target_include_directories(${PROJECT_NAME}_gz PUBLIC ".")
	target_include_directories(${PROJECT_NAME}_gz PUBLIC ${GAZEBO_INCLUDE_DIRS})
	target_include_directories(${PROJECT_NAME}_gz PUBLIC ${PRX_GZ_INCLUDE_DIRS})

	link_directories(${PRX_GZ_LIBRARY_DIRS})
	target_link_libraries(${PROJECT_NAME}_gz PUBLIC ${PROJECT_NAME}_utilities )
	target_link_libraries(${PROJECT_NAME}_gz PRIVATE ${GAZEBO_LIBRARIES})
	# message("GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")

endif(BUILD_FOR_GAZEBO)
