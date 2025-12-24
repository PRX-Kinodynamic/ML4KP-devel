## Utilities library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_SIMULATION RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_SIMULATION  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_SIMULATION}" PARENT_SCOPE)

if (${BUILD_STATIC_LIBRARY})
	add_library(Simulation STATIC ${SRC_PRX_LIB_SIMULATION})
else()
	add_library(Simulation ${SRC_PRX_LIB_SIMULATION})
endif()

target_include_directories(Simulation
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

target_link_libraries(Simulation INTERFACE Utilities )
# target_link_libraries(Simulation INTERFACE Planning )
# target_link_libraries(Simulation INTERFACE ${PROJECT_NAME}_external )
set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Simulation" PARENT_SCOPE)
