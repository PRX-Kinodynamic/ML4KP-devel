## External libraries of DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")

file(GLOB_RECURSE SRC_PRX_LIB_EXTERNAL_PQP  PQP/*.cpp)

add_library(${PROJECT_NAME}_external SHARED ${SRC_PRX_LIB_EXTERNAL_PQP} ) 
set(EXTERNAL_LINKING_LIBS "")
if(BUILD_QHULL)
	if(NOT ExternalProject)
	  # include(FetchContent)
	  include(ExternalProject)
	endif()
	SET(QHULL_SOURCE_DIR ${CMAKE_BINARY_DIR}/external/qhull/)
	SET(QHULL_BINARY_DIR ${CMAKE_BINARY_DIR}/external/qhull/build/)
	ExternalProject_add(
		QHull
		GIT_REPOSITORY https://github.com/qhull/qhull.git
    GIT_TAG 613debeaea72ee66626dace9ba1a2eff11b5d37d # v8.0.2
    GIT_PROGRESS TRUE
    SOURCE_DIR ${QHULL_SOURCE_DIR}
    BINARY_DIR ${QHULL_BINARY_DIR}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
    CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true"
    # CMAKE_CACHE_ARGS ""
    # CONFIGURE_COMMAND ""
    # BUILD_COMMAND ""
    # INSTALL_COMMAND "c"
    # TEST_COMMAND "ctest"
		)
	macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()
	# add_subdirectory(${CMAKE_BINARY_DIR}/external/qhull/)
	# message("qhull_LIBRARY ${QHULL_SOURCE_DIR}/QhullExport")
	# find_package(QHull REQUIRED PATHS ${QHULL_SOURCE_DIR}/QhullExport NO_DEFAULT_PATH)
	# print_all_variables()
	# SET(QHULL_LIBS ${QHULL_BINARY_DIR}/libqhull_r.dylib)
	# TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_external PUBLIC ${QHULL_SOURCE_DIR}/src/libqhullcpp)
	# TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_external PUBLIC ${QHULL_SOURCE_DIR}/src)
	# TARGET_LINK_DIRECTORIES(${PROJECT_NAME}_external PUBLIC ${QHULL_SOURCE_DIR}/build)
	# SET(QHULL_LIBS ${QHULL_BINARY_DIR}/libqhullstatic_r.a)
	# list(APPEND QHULL_LIBS ${QHULL_BINARY_DIR}/libqhullcpp.a)

	# message("QHULL_LIBS ${QHULL_LIBS}" )

	# add_library(QHull_lib SHARED IMPORTED)
	# set_target_properties(QHull_lib PROPERTIES IMPORTED_LOCATION ${QHULL_LIBS})
	# set_target_properties(ForexConnect PROPERTIES IMPORTED_LOCATION ${FXCM_LIB_DIR}/libForexConnect.so)

	# message("qhull_LIBRARY ${CMAKE_BINARY_DIR}/qhull-build/libqhull_r.dylib")
	# target_include_directories(${PROJECT_NAME}_external PUBLIC ${qhull_SOURCE_DIR}/qhull-src/src)
	# target_link_directories(${PROJECT_NAME}_external PUBLIC ${CMAKE_BINARY_DIR}/qhull-build/)
	# target_link_libraries(${PROJECT_NAME}_external PUBLIC QHull_lib)
	# target_link_libraries(${PROJECT_NAME}_external PUBLIC ${qhull_LIBRARIES} )
	# add_dependencies(${PROJECT_NAME}_external QHull)
	# target_link_libraries(${PROJECT_NAME}_external PUBLIC 
	# 	${QHULL_BINARY_DIR}/libqhull_r.8.0.2.dylib
	# 	${QHULL_BINARY_DIR}/libqhullstatic_r.a
	# 	)

	# set(qhull_static_r ${QHULL_BINARY_DIR}/libqhullstatic_r.a)
	set(qhull_static_r ${QHULL_BINARY_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}qhullstatic_r${CMAKE_STATIC_LIBRARY_SUFFIX})
	set(qhull_static ${QHULL_BINARY_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}qhullcpp${CMAKE_STATIC_LIBRARY_SUFFIX})
	
	add_library(libqhullstatic_r STATIC IMPORTED)
	set_property(TARGET libqhullstatic_r PROPERTY IMPORTED_LOCATION ${qhull_static_r})
	set_property(TARGET libqhullstatic_r PROPERTY IMPORTED_LOCATION_DEBUG ${qhull_static_r})
	set_property(TARGET libqhullstatic_r PROPERTY IMPORTED_LOCATION_RELEASE ${qhull_static_r})
	add_dependencies(libqhullstatic_r QHull)
	add_library(libqhullcpp STATIC IMPORTED)
	set_property(TARGET libqhullcpp PROPERTY IMPORTED_LOCATION ${qhull_static})
	set_property(TARGET libqhullcpp PROPERTY IMPORTED_LOCATION_DEBUG ${qhull_static})
	set_property(TARGET libqhullcpp PROPERTY IMPORTED_LOCATION_RELEASE ${qhull_static})
	set_property(TARGET libqhullcpp PROPERTY INTERFACE_LINK_LIBRARIES libqhullstatic_r)
	add_dependencies(libqhullcpp QHull)

	# target_link_libraries(${PROJECT_NAME}_external PUBLIC libqhullcpp)
	# target_link_libraries(${PROJECT_NAME}_external PUBLIC libqhullstatic_r)

	list(APPEND EXTERNAL_LINKING_LIBS libqhullcpp)
	list(APPEND EXTERNAL_LINKING_LIBS libqhullstatic_r)

endif(BUILD_QHULL)

target_include_directories(${PROJECT_NAME}_external PUBLIC ".")

# Silencing all warning for the external (non-prx) code
set_target_properties(${PROJECT_NAME}_external PROPERTIES COMPILE_FLAGS "-w")

if(BUILD_FOR_TORCH AND DEFINED ENV{Torch_DIR})
	# target_link_libraries(${PROJECT_NAME}_external ${TORCH_LIBRARIES})
	list(APPEND ${EXTERNAL_LINKING_LIBS} ${TORCH_LIBRARIES})
endif()

message(STATUS "EXTERNAL_LINKING_LIBS: ${EXTERNAL_LINKING_LIBS}")
target_link_libraries(${PROJECT_NAME}_external PUBLIC ${EXTERNAL_LINKING_LIBS})
