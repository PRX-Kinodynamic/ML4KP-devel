## Utilities library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_VISUALIZATION RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_VISUALIZATION  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_VISUALIZATION}" PARENT_SCOPE)

add_library(${PROJECT_NAME}_visualization ${SRC_PRX_LIB_VISUALIZATION})

target_include_directories(${PROJECT_NAME}_visualization PUBLIC ".")
target_link_libraries(${PROJECT_NAME}_visualization ${PROJECT_NAME}_simulation )
target_link_libraries(${PROJECT_NAME}_visualization ${PROJECT_NAME}_utilities )
set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${PROJECT_NAME}_visualization" PARENT_SCOPE)
