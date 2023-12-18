if(BUILD_FOR_FACTOR_GRAPHS)
	find_package(GTSAM REQUIRED) # Uses installed package
	find_package(GTSAMCMakeTools)
	set(GTSAM_LIBS gtsam)

	file(GLOB_RECURSE SRC_PRX_LIB_FACTOR_GRAPHS 
		planning/*.cpp
		utilities/*.cpp
		factors/*.cpp
		)
	add_library(${PROJECT_NAME}_factor_graphs STATIC ${SRC_PRX_LIB_FACTOR_GRAPHS})

	target_include_directories(${PROJECT_NAME}_factor_graphs PUBLIC ".")
	target_include_directories(${PROJECT_NAME}_factor_graphs PUBLIC ${GTSAM_INCLUDE_DIR})
	target_include_directories(${PROJECT_NAME}_factor_graphs PUBLIC ${SDFormat_INCLUDE_DIRS})
	link_directories(${SDFormat_LIBRARY_DIRS})
	target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC ${GTSAM_LIBS} ${SDFormat_LIBRARIES})

	message(STATUS "GTSAM_LIBS gtsam::Eigen3::Eigen")
	# target_link_libraries(${PROJECT_NAME}_factor_graphs INTERFACE fmt::fmt-header-only)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC TBB::tbb)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC tbb tbbmalloc)

endif(BUILD_FOR_FACTOR_GRAPHS)