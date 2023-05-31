## External libraries of DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")

file(GLOB_RECURSE SRC_PRX_LIB_EXTERNAL_PQP  PQP/*.cpp)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)

add_library(${PROJECT_NAME}_external ${SRC_PRX_LIB_EXTERNAL_PQP} ) 
target_include_directories(${PROJECT_NAME}_external PUBLIC ${Boost_INCLUDE_DIRS})

if (BUILD_CGAL)
	find_package(CGAL REQUIRED)
endif(BUILD_CGAL)

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
	set(QHULL_LIBS libqhullcpp)
	list(APPEND QHULL_LIBS libqhullstatic_r)
	target_link_libraries(${PROJECT_NAME}_external PUBLIC ${QHULL_LIBS})
	#target_link_libraries(${PROJECT_NAME}_external PUBLIC libqhullcpp)
	#target_link_libraries(${PROJECT_NAME}_external PUBLIC libqhullstatic_r)
	target_include_directories(${PROJECT_NAME}_external PUBLIC ${CMAKE_BINARY_DIR}/external/qhull/src )
else()
	message(STATUS "Not building Qhull")
    add_definitions(-DQHULL_NOT_BUILT)
endif(BUILD_QHULL)


target_include_directories(${PROJECT_NAME}_external PUBLIC ${Boost_INCLUDE_DIRS})
# link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}_external PUBLIC ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )



target_include_directories(${PROJECT_NAME}_external PUBLIC ".")


# Silencing all warning for the external (non-prx) code
set_target_properties(${PROJECT_NAME}_external PROPERTIES COMPILE_FLAGS "-w")
target_link_libraries(${PROJECT_NAME}_external PUBLIC ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

if (BUILD_CGAL)
	target_link_libraries(${PROJECT_NAME}_external PUBLIC CGAL::CGAL)
endif(BUILD_CGAL)

if(BUILD_FOR_TORCH AND DEFINED ENV{Torch_DIR})
	target_link_libraries(${PROJECT_NAME}_external ${TORCH_LIBRARIES})
endif()
