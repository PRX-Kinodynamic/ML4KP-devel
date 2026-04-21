if(BUILD_FOR_GTSAM)
	# add_definitions(-DGTSAM_USE_QUATERNIONS)
	
	# find_package(GTSAM REQUIRED) # Uses installed package
	# find_package(GTSAMCMakeTools)

	file(GLOB_RECURSE HDS_PRX_LIB_FACTOR_GRAPHS *.hpp)
	file(GLOB_RECURSE HEADER_PRX_LIB_FACTOR_GRAPHS RELATIVE ${CMAKE_SOURCE_DIR}/src ${HDS_PRX_LIB_FACTOR_GRAPHS})

	set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_FACTOR_GRAPHS}" PARENT_SCOPE)

	# add_library(FactorGraphs INTERFACE ${HDS_PRX_LIB_FACTOR_GRAPHS})
	# add_library(FactorGraphs INTERFACE IMPORTED ${HDS_PRX_LIB_FACTOR_GRAPHS} GLOBAL)
	add_library(FactorGraphs INTERFACE  ${HDS_PRX_LIB_FACTOR_GRAPHS})
	
	set_target_properties(FactorGraphs PROPERTIES LINKER_LANGUAGE CXX)

	# get_target_property(GTSAM_INCLUDE_DIRECTORIES gtsam INTERFACE_INCLUDE_DIRECTORIES)
	target_include_directories(FactorGraphs INTERFACE
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)
	# target_link_libraries(FactorGraphs INTERFACE gtsam)
														# $<BUILD_INTERFACE:gtsam>
                            # $<INSTALL_INTERFACE:gtsam>
	# 													"$<BUILD_INTERFACE:gtsam>"
	# 	)

	# message(STATUS "GTSAM_USE_TBB: ${GTSAM_USE_TBB}")
	# if (${GTSAM_USE_TBB})
	# 	message(STATUS "GTSAM built with TBB")

		# target_link_libraries(FactorGraphs INTERFACE ${GTSAM_ADDITIONAL_LIBRARIES})
	# endif()
	# message(STATUS "GTSAM_LIBS gtsam::Eigen3::Eigen")
	# target_link_libraries(${PROJECT_NAME}_factor_graphs INTERFACE fmt::fmt-header-only)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC TBB::tbb)
	# target_link_libraries(${PROJECT_NAME}_factor_graphs PUBLIC tbb tbbmalloc)
	# set(${PROJECT_NAME}_ADDITIONAL_LIBRARIES "gtsam;${GTSAM_ADDITIONAL_LIBRARIES}" PARENT_SCOPE)
	set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};FactorGraphs;" PARENT_SCOPE)

endif(BUILD_FOR_GTSAM)
