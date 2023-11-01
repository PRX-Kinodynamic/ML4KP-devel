## Utilities library of DirtMP
find_package(YAMLCPP NAMES Yaml-cpp YamlCpp yaml-cpp)

include_directories(${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS})

file(GLOB_RECURSE HEADER_PRX_LIB_UTILITIES RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_UTILITIES  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_UTILITIES}" PARENT_SCOPE)

add_library(${PROJECT_NAME}_utilities ${SRC_PRX_LIB_UTILITIES})

target_include_directories(${PROJECT_NAME}_utilities PUBLIC ".")

set_target_properties(${PROJECT_NAME}_utilities PROPERTIES COMPILE_FLAGS "-Wno-deprecated")

if (APPLE)
	list(APPEND YAML_CPP_LIBRARIES yaml-cpp)
endif(APPLE)

target_link_libraries(${PROJECT_NAME}_utilities PUBLIC ${PROJECT_NAME}_external )
target_link_libraries(${PROJECT_NAME}_utilities PUBLIC ${YAMLCPP_LIBRARY} ${YAML_CPP_LIBRARIES})

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${PROJECT_NAME}_utilities" PARENT_SCOPE)
