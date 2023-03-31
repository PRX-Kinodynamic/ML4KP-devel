## Planning library of DirtMP

file(GLOB_RECURSE SRC_PRX_LIB_PLANNING  ./*.cpp)

add_library(${PROJECT_NAME}_planning SHARED ${SRC_PRX_LIB_PLANNING})

target_include_directories(${PROJECT_NAME}_planning PUBLIC ".")

target_link_libraries(${PROJECT_NAME}_planning PUBLIC ${PROJECT_NAME}_utilities )
target_link_libraries(${PROJECT_NAME}_planning PUBLIC ${PROJECT_NAME}_simulation )
