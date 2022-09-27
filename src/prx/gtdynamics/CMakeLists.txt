

if(BUILD_FOR_GTDYNAMICS)
	if(UNIX)
    # find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
		# find_package(TBB REQUIRED)
	# 	add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
	endif()
	# add_subdirectory(GTDynamics/)
	set(SDFormat_VERSION 12)
	find_package(sdformat${SDFormat_VERSION} REQUIRED)
	find_package(GTSAM REQUIRED) # Uses installed package
	find_package(GTSAMCMakeTools)
	# find_package(gtdynamics REQUIRED)
	set(GTSAM_LIBS gtsam)
	# set(gtdynamics_include_dir "${gtdynamics_DIR}/../../../include")
	# set(gtdynamics_libs "${gtdynamics_DIR}/../../libgtdynamics.a")
	# set(gtdynamics_include_dir "${gtdynamics_DIR}/install/usr/local/include/")
	# set(gtdynamics_libs "${gtdynamics_DIR}/install/lib/libgtdynamics.so")
	# message("include dirs: ${gtdynamics_include_dir}")
	# message("libs: ${gtdynamics_libs}")
	PRINT_CMAKE_VARS(gt)
	# set(GTSAM_INCLUDE_DIR )
	# include_directories(${GTSAM_INCLUDE_DIR})
	# include_directories(${gtdynamics_include_dir})

	# link_directories(${SDFormat_LIBRARY_DIRS})
	# PRINT_CMAKE_VARS(sources)
	# PRINT_CMAKE_VARS(gt)

	file(GLOB_RECURSE SRC_PRX_LIB_GTDYNAMICS 
		planning/*.cpp
		utilities/*.cpp
		factors/*.cpp
		)
	add_library(${PROJECT_NAME}_gtdynamics STATIC ${SRC_PRX_LIB_GTDYNAMICS})

	target_include_directories(${PROJECT_NAME}_gtdynamics PUBLIC ".")
	target_include_directories(${PROJECT_NAME}_gtdynamics PUBLIC ${GTSAM_INCLUDE_DIR})
	target_include_directories(${PROJECT_NAME}_gtdynamics PUBLIC ${gtdynamics_include_dir} ${SDFormat_INCLUDE_DIRS})
	link_directories(${SDFormat_LIBRARY_DIRS})
	target_link_libraries(${PROJECT_NAME}_gtdynamics PUBLIC ${GTSAM_LIBS} ${SDFormat_LIBRARIES} ${gtdynamics_libs})
	# target_link_libraries(${PROJECT_NAME}_gtdynamics PUBLIC TBB::tbb)
	# target_link_libraries(${PROJECT_NAME}_gtdynamics PUBLIC tbb tbbmalloc)

endif(BUILD_FOR_GTDYNAMICS)