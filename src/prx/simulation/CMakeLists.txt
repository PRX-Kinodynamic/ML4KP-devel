## Utilities library of DirtMP

file(GLOB_RECURSE SRC_PRX_LIB_SIMULATION  ./*.cpp)

add_library(${PROJECT_NAME}_simulation ${SRC_PRX_LIB_SIMULATION})

target_include_directories(${PROJECT_NAME}_simulation PUBLIC ".")

target_link_libraries(${PROJECT_NAME}_simulation PUBLIC ${PROJECT_NAME}_utilities )
target_link_libraries(${PROJECT_NAME}_simulation PUBLIC ${PROJECT_NAME}_external )