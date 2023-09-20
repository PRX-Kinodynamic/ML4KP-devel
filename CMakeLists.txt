cmake_minimum_required(VERSION 3.5)
project(DirtMP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

cmake_policy(SET CMP0079 NEW)

set(BUILD_FOR_PYTHON OFF CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_TORCH  ON CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_FOR_MUJOCO ON CACHE BOOL "Set ON/TRUE for building using MuJoCo")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")

if( NOT CMAKE_BUILD_TYPE )
	set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build types" FORCE)
endif()

if (CMAKE_BUILD_TYPE MATCHES Release)
	set(BUILD_FLAGS "-O3")
    add_definitions(-DRELEASE_BUILD)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
	set(BUILD_FLAGS "-O0 -fno-inline -g")
    add_definitions(-DDEBUG_BUILD)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

add_definitions(${BUILD_FLAGS})

find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(${Boost_INCLUDE_DIRS})
link_libraries(
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
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

include_directories(src/prx/external)
include_directories(src/prx/visualization)
include_directories(src/prx/planning)
include_directories(src/prx/utilities)
include_directories(src/prx/simulation)

# TODO: Make a shared libraries.
add_subdirectory(src/prx/planning)
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/visualization)

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

add_subdirectory(examples)

if(BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif(BUILD_TESTS)


