
file(GLOB_RECURSE SRC_PRX_LIB_PLUGINS  
		*.cpp
	)
# include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${PRX_GZ_INCLUDE_DIRS})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
# target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})
# set(LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY}/gazebo_plugins)
	message("plugins PRX_GZ_INCLUDE_DIRS: ${PRX_GZ_INCLUDE_DIRS}")

foreach(f ${SRC_PRX_LIB_PLUGINS})
	message("Plugin: ${f}")
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	# add_executable(gazebo_plugins/${F_EXEC} ${f})
	add_library(${F_EXEC} MODULE ${f})

	set_target_properties(${F_EXEC} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}/gazebo_plugins)
	target_include_directories(${F_EXEC} PUBLIC ${GAZEBO_INCLUDE_DIRS})
	target_include_directories(${F_EXEC} PUBLIC ${PRX_GZ_INCLUDE_DIRS})

	target_link_libraries(${F_EXEC} PUBLIC ${PRX_GZ_LIBRARIES})
	target_link_libraries(${F_EXEC} PRIVATE ${GAZEBO_LIBRARIES})
	target_link_libraries(${F_EXEC} PRIVATE ${PROJECT_NAME}_GZ_MSGS )


endforeach()