file(GLOB EXECUTABLES_TRIPODS_FILES *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tripods )
	
if(BUILD_FOR_GTSAM)

	foreach(f ${EXECUTABLES_TRIPODS_FILES})
		GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
		add_executable(${F_EXEC} ${f})
		target_link_libraries(${F_EXEC} ${PROJECT_NAME} gtsam)
	endforeach()

endif(BUILD_FOR_GTSAM)