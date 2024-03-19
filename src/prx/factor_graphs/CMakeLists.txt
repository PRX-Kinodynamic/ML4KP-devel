if(BUILD_FOR_GTSAM)
	add_definitions(-DGTSAM_USE_QUATERNIONS)
	
	find_package(GTSAM REQUIRED) # Uses installed package
	# find_package(GTSAMCMakeTools)

	file(GLOB_RECURSE SRC_PRX_LIB_FACTOR_GRAPHS *.cpp)
	file(GLOB_RECURSE HDS_PRX_LIB_FACTOR_GRAPHS *.hpp)
	file(GLOB_RECURSE HEADER_PRX_LIB_FACTOR_GRAPHS RELATIVE ${CMAKE_SOURCE_DIR}/src ${HDS_PRX_LIB_FACTOR_GRAPHS})

	set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_FACTOR_GRAPHS}" PARENT_SCOPE)

	add_library(FactorGraphs INTERFACE)
	set_target_properties(FactorGraphs PROPERTIES LINKER_LANGUAGE CXX)

	message(STATUS "GTSAM_DIR: ${GTSAM_DIR}")
	message(STATUS "GTSAM_CONFIG: ${GTSAM_CONFIG}")
	target_include_directories(FactorGraphs INTERFACE
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}:${GTSAM_INCLUDE_DIR}>"
														)
	target_link_libraries(FactorGraphs INTERFACE gtsam)
	
	# message(STATUS "GTSAM_LIBS gtsam::Eigen3::Eigen")
	# target_link_libraries(${PROJECT_NAME}_factor_graphs INTERFACE fmt::fmt-header-only)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC TBB::tbb)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC tbb tbbmalloc)
	set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};FactorGraphs" PARENT_SCOPE)

endif(BUILD_FOR_GTSAM)