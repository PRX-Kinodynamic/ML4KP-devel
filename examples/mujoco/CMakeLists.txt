file(GLOB_RECURSE EXAMPLES_MUJOCO_FILES *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/mujoco )
	
foreach(f ${EXAMPLES_MUJOCO_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC} ${PROJECT_NAME})
    if (BUILD_FOR_MUJOCO)
        target_link_libraries(${F_EXEC} ${PROJECT_NAME}_mjsim ${OpenCV_LIBS} )
    endif()
endforeach()