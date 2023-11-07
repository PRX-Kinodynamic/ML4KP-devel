## Utilities library of DirtMP
find_package(YAMLCPP NAMES Yaml-cpp YamlCpp)
include_directories(${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS})

file(GLOB_RECURSE HEADER_PRX_LIB_UTILITIES RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_UTILITIES  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_UTILITIES}" PARENT_SCOPE)

add_library(Utilities ${SRC_PRX_LIB_UTILITIES})

target_include_directories(Utilities PUBLIC
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

set_target_properties(Utilities PROPERTIES COMPILE_FLAGS "-Wno-deprecated")

# if (APPLE)
# 	list(APPEND YAML_CPP_LIBRARIES yaml-cpp)
# endif(APPLE)

target_link_libraries(Utilities INTERFACE External )
target_link_libraries(Utilities PUBLIC yaml-cpp::yaml-cpp)

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Utilities" PARENT_SCOPE)
