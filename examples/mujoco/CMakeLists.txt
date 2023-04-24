
if(BUILD_FOR_MUJOCO)
    file(GLOB_RECURSE MJ_EXAMPLE_FILES *.cpp)

    set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/mujoco)

    foreach(f ${MJ_EXAMPLE_FILES})
        get_filename_component(EXAMPLE_NAME ${f} NAME_WE)
    
        add_executable(${EXAMPLE_NAME} ${f})
    
        target_link_libraries(${EXAMPLE_NAME} 
            ${PROJECT_NAME}_external 
            ${PROJECT_NAME}_utilities 
            ${PROJECT_NAME}_simulation 
            ${PROJECT_NAME}_planning 	
            ${PROJECT_NAME}_visualization 
            ${PROJECT_NAME}_mjsim
        )
    endforeach()
endif()