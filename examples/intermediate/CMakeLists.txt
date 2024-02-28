file(GLOB_RECURSE EXAMPLES_INTERMEDIATE_FILES
    *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/intermediate )

foreach(f ${EXAMPLES_INTERMEDIATE_FILES})
    GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
    add_executable(${F_EXEC} ${f})
    target_link_libraries(${F_EXEC} ${PROJECT_NAME})
    if (BUILD_FOR_MUJOCO)
        target_link_libraries(${F_EXEC} ${PROJECT_NAME}_mjsim )
    endif()
endforeach()