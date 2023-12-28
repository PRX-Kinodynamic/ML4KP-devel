## Planning library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_PLANNING RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_PLANNING  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_PLANNING}" PARENT_SCOPE)

add_library(Planning  ${SRC_PRX_LIB_PLANNING})

target_include_directories(Planning PUBLIC 
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

target_link_libraries(Planning INTERFACE Simulation )

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Planning" PARENT_SCOPE)
