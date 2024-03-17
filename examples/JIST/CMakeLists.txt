file(GLOB_RECURSE EXAMPLES_JIST_FILES
    *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/JIST )

message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
foreach(f ${EXAMPLES_JIST_FILES})
    GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
    add_executable(${F_EXEC} ${f})
    target_link_libraries(${F_EXEC} ${PROJECT_NAME})
    if (BUILD_FOR_MUJOCO)
        target_link_libraries(${F_EXEC} ${PROJECT_NAME}_mjsim ${OpenCV_LIBS} )
    endif()
endforeach()