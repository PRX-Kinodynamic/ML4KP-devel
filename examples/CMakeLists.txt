
set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/examples )

add_subdirectory(basic)
add_subdirectory(intermediate)
add_subdirectory(advanced)
add_subdirectory(tripods)
if (BUILD_FOR_MUJOCO)
    add_subdirectory(mujoco)
endif()