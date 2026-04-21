## Utilities library of DirtMP

file(GLOB_RECURSE HEADER_PRX_LIB_UTILITIES RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)
file(GLOB_RECURSE SRC_PRX_LIB_UTILITIES  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_UTILITIES}" PARENT_SCOPE)

if (${BUILD_STATIC_LIBRARY})
	add_library(Utilities STATIC ${SRC_PRX_LIB_UTILITIES})
else()
	add_library(Utilities ${SRC_PRX_LIB_UTILITIES})
endif()

target_include_directories(Utilities PUBLIC
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

set_target_properties(Utilities PROPERTIES COMPILE_FLAGS "-Wno-deprecated")

target_link_libraries(Utilities PUBLIC External )

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Utilities" PARENT_SCOPE)
