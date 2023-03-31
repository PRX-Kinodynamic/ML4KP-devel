

if(BUILD_FOR_FACTOR_GRAPHS)
	if(UNIX)
    # find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
		# find_package(TBB REQUIRED)
	# 	add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
	endif()
	# add_subdirectory(factor_graphs/)
	# set(SDFormat_VERSION 12)
	# find_package(sdformat${SDFormat_VERSION} REQUIRED)
	# find_package(Eigen3 REQUIRED)
	# add_library(Eigen3::Eigen ALIAS Eigen3)
	find_package(GTSAM REQUIRED) # Uses installed package
	find_package(GTSAMCMakeTools)
	# find_library(gtsam_eigen gtsam_eigen3)
	# find_package(factor_graphs REQUIRED)
	set(GTSAM_LIBS gtsam)
	# set(factor_graphs_include_dir "${factor_graphs_DIR}/../../../include")
	# set(factor_graphs_libs "${factor_graphs_DIR}/../../libfactor_graphs.a")
	# set(factor_graphs_include_dir "${factor_graphs_DIR}/install/usr/local/include/")
	# set(factor_graphs_libs "${factor_graphs_DIR}/install/lib/libfactor_graphs.so")
	# message("include dirs: ${factor_graphs_include_dir}")
	# message("libs: ${factor_graphs_libs}")
	PRINT_CMAKE_VARS(gt)
	# set(GTSAM_INCLUDE_DIR )
	# include_directories(${GTSAM_INCLUDE_DIR})
	# include_directories(${factor_graphs_include_dir})

	# link_directories(${SDFormat_LIBRARY_DIRS})
	# PRINT_CMAKE_VARS(sources)
	# PRINT_CMAKE_VARS(gt)

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