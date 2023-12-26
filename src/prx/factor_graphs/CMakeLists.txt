if(BUILD_FOR_FACTOR_GRAPHS)
	find_package(GTSAM REQUIRED) # Uses installed package
	# find_package(GTSAMCMakeTools)

	file(GLOB_RECURSE HEADER_PRX_LIB_FACTOR_GRAPHS RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
	file(GLOB_RECURSE SRC_PRX_LIB_FACTOR_GRAPHS *.cpp)

	set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_FACTOR_GRAPHS}" PARENT_SCOPE)

	add_library(FactorGraphs ${SRC_PRX_LIB_FACTOR_GRAPHS})
	set_target_properties(FactorGraphs PROPERTIES LINKER_LANGUAGE CXX)

	target_include_directories(FactorGraphs PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}:${GTSAM_INCLUDE_DIR}>"
														)
	target_link_libraries(FactorGraphs INTERFACE gtsam)
	
	# message(STATUS "GTSAM_LIBS gtsam::Eigen3::Eigen")
	# target_link_libraries(${PROJECT_NAME}_factor_graphs INTERFACE fmt::fmt-header-only)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC TBB::tbb)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC tbb tbbmalloc)
	set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};FactorGraphs" PARENT_SCOPE)

endif(BUILD_FOR_FACTOR_GRAPHS)