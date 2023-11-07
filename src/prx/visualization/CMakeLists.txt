## Utilities library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_VISUALIZATION RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_VISUALIZATION  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_VISUALIZATION}" PARENT_SCOPE)

add_library(Visualization ${SRC_PRX_LIB_VISUALIZATION})

target_include_directories(Visualization 	
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

target_link_libraries(Visualization Simulation )
target_link_libraries(Visualization Utilities )
set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Visualization" PARENT_SCOPE)
