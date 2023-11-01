## Utilities library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_SIMULATION RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_SIMULATION  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_SIMULATION}" PARENT_SCOPE)

add_library(${PROJECT_NAME}_simulation ${SRC_PRX_LIB_SIMULATION})

target_include_directories(${PROJECT_NAME}_simulation PUBLIC ".")

target_link_libraries(${PROJECT_NAME}_simulation PUBLIC ${PROJECT_NAME}_utilities )
# target_link_libraries(${PROJECT_NAME}_simulation INTERFACE ${PROJECT_NAME}_external )
set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${PROJECT_NAME}_simulation" PARENT_SCOPE)
