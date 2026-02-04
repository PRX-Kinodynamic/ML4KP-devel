cmake_minimum_required(VERSION 3.5)
project(ML4KP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_CXX_STANDARD 17)

cmake_policy(SET CMP0079 NEW)

set(BUILD_FOR_PYTHON OFF CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_TORCH  OFF CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_FOR_MUJOCO ON CACHE BOOL "Set ON/TRUE for building using MuJoCo")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")
set(BUILD_FOR_GTSAM ON CACHE BOOL "Set ON/TRUE for building GTSAM factor graphs")
set(BUILD_WITH_SIMD OFF CACHE BOOL "Set ON/TRUE for building with SIMD (vectorized) operations")
set(BUILD_STATIC_LIBRARY OFF CACHE BOOL "Set ON/TRUE for building static library")

set(${PROJECT_NAME}_VERSION 1.0)
set(${PROJECT_NAME}_HEADERS "" CACHE STRING "Headers to be exported")
set(${PROJECT_NAME}_LIBRARIES "" CACHE STRING "Libraries to be exported")
set(THREADS_PREFER_PTHREAD_FLAG ON)


if( NOT CMAKE_BUILD_TYPE )
	set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build types" FORCE)
endif()

if (CMAKE_BUILD_TYPE MATCHES Release)
	# -03 <- Turn on all optims
	# -march=native <- Turn on eigen vectorization optimizations
	# set(BUILD_FLAGS "-O3 -march=native")
	set(BUILD_FLAGS "-O3")
    add_definitions(-DRELEASE_BUILD)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
	set(BUILD_FLAGS "-O0 -fno-inline -g")
    # list(APPEND BUILD_FLAGS "-fsanitize=address")
    add_definitions(-DDEBUG_BUILD)
endif()

if(BUILD_WITH_SIMD)
	list(APPEND BUILD_FLAGS "-march=native")
endif(BUILD_WITH_SIMD)

if (UNIX)
	if (NOT APPLE)
		# message(STATUS "Building with -fPIC")
    list(APPEND BUILD_FLAGS "-fPIC")
	endif ()
endif (UNIX)

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

add_definitions(${BUILD_FLAGS})

find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 CONFIG REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

find_package(Threads REQUIRED)
# link_libraries(Threads::Threads)

if(BUILD_FOR_TORCH)
	if(DEFINED ENV{Torch_DIR})
        find_package(Torch REQUIRED)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
    else()
		message("Please download libtorch C++, place in external folder and setup the enviornmental variable Torch_DIR accordingly.")
        add_definitions(TORCH_NOT_BUILT)
    endif()
else()
    add_definitions(-DTORCH_NOT_BUILT)
endif()

if(BUILD_FOR_GTSAM)
    # set(GTSAM_ALLOCATOR_TBB 1)
	# add_definitions(GTSAM_ALLOCATOR_TBB)
	find_package(GTSAM REQUIRED) # Uses installed package
	if(GTSAM_USE_TBB)
  	message(STATUS "GTSAM is using TBB")
    find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
	endif(GTSAM_USE_TBB)
endif(BUILD_FOR_GTSAM)

include_directories(src/prx/external)
include_directories(src/prx/visualization)
include_directories(src/prx/simulation)
include_directories(src/prx/planning)
include_directories(src/prx/utilities)

# TODO: Make a shared libraries.
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/planning)
add_subdirectory(src/prx/visualization)
add_subdirectory(src/prx/factor_graphs)

if(BUILD_FOR_MUJOCO)
	message("MuJoCo found at $ENV{MJ_PATH}")
	include_directories(src/prx/mujoco)
	add_subdirectory(src/prx/mujoco)
else()
	message("Not building with MuJoCo.")
	add_definitions(-DMUJOCO_NOT_BUILT)
endif()

if (BUILD_FOR_PYTHON)
	add_subdirectory(src/pyprx)
endif()

link_directories(${PROJECT_SOURCE_DIR}/lib)

# Create a single library, mostly for installing
add_library(${PROJECT_NAME} INTERFACE )
foreach(LIBRARY IN ITEMS ${${PROJECT_NAME}_LIBRARIES} )
	add_dependencies(${PROJECT_NAME} ${LIBRARY})
	target_link_libraries(${PROJECT_NAME} INTERFACE ${LIBRARY})
endforeach()

add_subdirectory(examples)
add_subdirectory(executables)

if(BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif(BUILD_TESTS)


# Install the library
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)



set(CMAKE_INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/")
message(STATUS "Installing ${PROJECT_NAME} in: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")

foreach(HEADER IN ITEMS ${${PROJECT_NAME}_HEADERS})
	get_filename_component(H_DIR ${HEADER} DIRECTORY )
	install(FILES src/${HEADER} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${H_DIR})
endforeach()

# install(TARGETS ${${PROJECT_NAME}_LIBRARIES} EXPORT ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
foreach(LIBRARY IN ITEMS ${${PROJECT_NAME}_LIBRARIES})
	install(TARGETS ${LIBRARY}
        EXPORT ${LIBRARY}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	)
	install(EXPORT ${LIBRARY}Targets
        	FILE ${LIBRARY}Targets.cmake
	        NAMESPACE ${PROJECT_NAME}::
	        DESTINATION ${CMAKE_INSTALL_DESTINATION}
	)
	# export(EXPORT ${LIBRARY}
    #     FILE ${CMAKE_INSTALL_DESTINATION}/${LIBRARY}Config.cmake
    #     NAMESPACE ${PROJECT_NAME}::)

endforeach()
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        EXPORT_LINK_INTERFACE_LIBRARIES
        DESTINATION ${CMAKE_INSTALL_DESTINATION}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ML4KPConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(cmake/ML4KPConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/ML4KPConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DESTINATION}
  PATH_VARS
  	CMAKE_INSTALL_INCLUDEDIR
  	CMAKE_INSTALL_LIBDIR 
  	${PROJECT_NAME}_LIBRARIES
  	BUILD_FOR_TORCH
  	BUILD_FOR_GTSAM)

install(FILES 
			"${CMAKE_CURRENT_BINARY_DIR}/cmake/ML4KPConfig.cmake"
  			"${CMAKE_CURRENT_BINARY_DIR}/ML4KPConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DESTINATION} )
