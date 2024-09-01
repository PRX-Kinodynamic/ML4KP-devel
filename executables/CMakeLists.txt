set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/executables )

add_subdirectory(factor_graphs)
add_subdirectory(tripods)
# add_subdirectory(perception)

file(GLOB EXECUTABLE_FILES *.cpp)

foreach(f ${EXECUTABLE_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC} ${PROJECT_NAME})
endforeach()
