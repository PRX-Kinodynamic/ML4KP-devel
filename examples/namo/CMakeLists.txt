file(GLOB_RECURSE EXAMPLES_NAMO_FILES *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/namo_v2 )

foreach(f ${EXAMPLES_NAMO_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC} ${PROJECT_NAME} nlohmann_json::nlohmann_json zmq yaml-cpp)
    if (BUILD_FOR_MUJOCO)
        target_link_libraries(${F_EXEC} ${PROJECT_NAME}_mjsim ${OpenCV_LIBS} )
    endif()
    # Add ZeroMQ linking when ZeroMQ is found
    # if (ZeroMQ_FOUND)
    #     target_link_libraries(${F_EXEC} ${ZeroMQ_LIBRARIES})
    # endif()
endforeach()