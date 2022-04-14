cmake_minimum_required(VERSION 3.5)
project(DirtMP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(BUILD_SHARED_LIBS True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
cmake_policy(SET CMP0079 NEW)

set(BUILD_FOR_PYTHON OFF CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_BULLET OFF CACHE BOOL "Set ON/TRUE for building for PyBullet")
set(BUILD_FOR_TORCH  ON CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")

set(BUILD_FOR_GTDYNAMICS ON CACHE BOOL "Set ON/TRUE for building tests")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(cmake_utils)

add_definitions(-O3 -fPIC -g )
if(APPLE)
	set(CMAKE_CXX_STANDARD 17)
else()
	add_definitions(-lstdc++fs)
	link_libraries(stdc++fs)
endif()
find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
# link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})
link_libraries(
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	# ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
	)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

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


if(BUILD_FOR_BULLET)
	include_directories(src/prx/bullet_sim)
	add_subdirectory(src/prx/bullet_sim)
else()
	message("Not building for bullet")
    add_definitions(-DBULLET_NOT_BUILT)
endif()

include_directories(src/prx/external)
include_directories(src/prx/visualization)
include_directories(src/prx/planning)
include_directories(src/prx/utilities)
include_directories(src/prx/simulation)

if(BUILD_FOR_GAZEBO)
	include(src/prx/gazebo/gazebo.cmake)
	include_directories(src/prx/gazebo)
	add_subdirectory(src/prx/gazebo)
endif(BUILD_FOR_GAZEBO)

# TODO: Make a shared libraries.
add_subdirectory(src/prx/planning)
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/visualization)
add_subdirectory(src/prx/gtdynamics)

if (BUILD_FOR_PYTHON)
	enable_testing()
	add_subdirectory(src/pyprx)
endif()

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(examples)
add_subdirectory(executables)

if(BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif(BUILD_TESTS)
	# ${PROJECT_SOURCE_DIR}/tests/*.cpp
