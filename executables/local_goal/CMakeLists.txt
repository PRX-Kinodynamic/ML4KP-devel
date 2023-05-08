file(GLOB_RECURSE EXAMPLES_FILES *.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/local_goal )

ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

foreach(f ${EXAMPLES_FILES})
    GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
    add_executable(${F_EXEC} ${f})
    target_link_libraries(${F_EXEC}
        ${PROJECT_NAME}_external 
        ${PROJECT_NAME}_utilities 
        ${PROJECT_NAME}_simulation 
        ${PROJECT_NAME}_planning 	
        ${PROJECT_NAME}_visualization )
    if (BUILD_FOR_TORCH)
        target_link_libraries(${F_EXEC} ${TORCH_LIBRARIES})
    endif(BUILD_FOR_TORCH)
endforeach()