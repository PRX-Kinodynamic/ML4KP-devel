## External libraries of DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")

file(GLOB_RECURSE HEADER_PRX_LIB_EXTERNAL_PQP RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp ./*.h)
file(GLOB_RECURSE SRC_PRX_LIB_EXTERNAL_PQP  PQP/*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_EXTERNAL_PQP}" PARENT_SCOPE)


# if(BUILD_FOR_GTSAM)
# 	add_definitions(-DGTSAM_USE_QUATERNIONS)
# 	find_package(GTSAM REQUIRED)
# 	find_package(GTSAMCMakeTools REQUIRED)
# 	message(STATUS "GTSAM_DIR: ${GTSAM_DIR}")
# 	message(STATUS "GTSAM_CONFIG: ${GTSAM_CONFIG}")

# endif(BUILD_FOR_GTSAM)

add_library(External ${SRC_PRX_LIB_EXTERNAL_PQP} ) 


target_include_directories(External 
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

# Silencing all warning for the external (non-prx) code
set_target_properties(External PROPERTIES COMPILE_FLAGS "-w")

if(BUILD_FOR_TORCH AND DEFINED ENV{Torch_DIR})
	message(STATUS "Linking Torch: ${TORCH_LIBRARIES}")
	target_link_libraries(External PUBLIC ${TORCH_LIBRARIES})
endif()
target_link_libraries(External PUBLIC Boost::filesystem Boost::system Threads::Threads)

# if(BUILD_FOR_GTSAM)
	# target_link_libraries(External PUBLIC gtsam)
			# get_property(OUT T gtsam LINK_LIBRARIES)
		# message("OUT ${OUT}")
# endif(BUILD_FOR_GTSAM)

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};External" PARENT_SCOPE)
