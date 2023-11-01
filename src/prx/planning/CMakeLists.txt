## Planning library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_PLANNING RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_PLANNING  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_PLANNING}" PARENT_SCOPE)

add_library(${PROJECT_NAME}_planning  ${SRC_PRX_LIB_PLANNING})

target_include_directories(${PROJECT_NAME}_planning PUBLIC ".")

target_link_libraries(${PROJECT_NAME}_planning PUBLIC ${PROJECT_NAME}_simulation )

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${PROJECT_NAME}_planning" PARENT_SCOPE)
