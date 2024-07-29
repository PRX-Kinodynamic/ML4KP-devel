if(BUILD_FOR_TORCH)
    file(GLOB_RECURSE ROADMAPS_FILES *.cpp)

    set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/roadmaps)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

    foreach(f ${ROADMAPS_FILES})
        GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
        add_executable(${F_EXEC} ${f})
        target_link_libraries(${F_EXEC}
            ${PROJECT_NAME}_external 
            ${PROJECT_NAME}_utilities 
            ${PROJECT_NAME}_simulation 
            ${PROJECT_NAME}_planning 	
            ${PROJECT_NAME}_visualization 
            ${PROJECT_NAME}_mjsim
            ${TORCH_LIBRARIES}
            )
    endforeach()

endif(BUILD_FOR_TORCH)