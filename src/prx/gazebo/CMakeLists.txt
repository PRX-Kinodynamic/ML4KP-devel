## gazebo submodule of DirtMP

if(BUILD_FOR_GAZEBO)

	# find_package(Protobuf REQUIRED)
	# find_package(gazebo REQUIRED)
    find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

	# set(PRX_GZ_LIBRARIES)
	# define_property(GLOBAL PROPERTY PRX_GZ_INCLUDE_DIRS
 #                 BRIEF_DOCS "PRX + GZ include directories"
 #                 FULL_DOCS "PRX + GZ include directories")
	# set_property(GLOBAL PROPERTY PRX_GZ_LIBRARIES)
	# set_property(GLOBAL PROPERTY PRX_GZ_INCLUDE_DIRS)
	set(PRX_GZ_INCLUDE_DIRS "${GAZEBO_LIBRARY_DIRS}" 
		CACHE INTERNAL "PRX + GZ include directories" FORCE)
	set(PRX_GZ_LIBRARIES "" 
		CACHE INTERNAL "PRX + GZ include directories" FORCE)
	# set(GZ_CACHE_LIBRARIES "${GAZEBO_LIBRARIES}" 
	# 	CACHE INTERNAL "GAZEBO LIBRARIES" FORCE)
	# list(APPEND PRX_GZ_LIBRARIES ${GAZEBO_LIBRARIES})
	# find_ign_lib(ignition-common 3 1)
	# find_package(ignition-common3 QUIET)
	# find_package(ignition-msgs4 QUIET)
	# find_package(ignition-msgs5 QUIET)
	# find_package(ignition-transport7 QUIET)
	# find_package(ignition-transport8 QUIET)

	file(GLOB_RECURSE SRC_PRX_LIB_GZ_SIM  
		gz_sim.cpp
		plants/*.cpp
	)

	add_library(${PROJECT_NAME}_gz ${SRC_PRX_LIB_GZ_SIM})

	add_subdirectory(msgs)
	add_subdirectory(plugins)

	message("PRX_GZ_INCLUDE_DIRS: ${PRX_GZ_INCLUDE_DIRS}")

	link_directories(${PRX_GZ_LIBRARY_DIRS})

	target_include_directories(${PROJECT_NAME}_gz PUBLIC ".")
	target_include_directories(${PROJECT_NAME}_gz PUBLIC ${GAZEBO_INCLUDE_DIRS})
	target_include_directories(${PROJECT_NAME}_gz PUBLIC ${PRX_GZ_INCLUDE_DIRS})
	# target_include_directories(${PROJECT_NAME}_gz PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/msgs)
	
	target_link_libraries(${PROJECT_NAME}_gz PRIVATE ${GAZEBO_LIBRARIES} )

	target_link_libraries(${PROJECT_NAME}_gz PUBLIC ${PRX_GZ_LIBRARIES})
	target_link_libraries(${PROJECT_NAME}_gz PUBLIC ${PROJECT_NAME}_utilities )
	target_link_libraries(${PROJECT_NAME}_gz PUBLIC ${PROJECT_NAME}_simulation )
	target_link_libraries(${PROJECT_NAME}_gz PUBLIC tbb tbbmalloc)
	# message("GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")

endif(BUILD_FOR_GAZEBO)
