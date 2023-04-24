cmake_minimum_required(VERSION 3.5)
project(DirtMP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(BUILD_SHARED_LIBS True)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
cmake_policy(SET CMP0079 NEW)

set(BUILD_FOR_PYTHON OFF CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_TORCH  ON CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")
set(PY_VERSION "3" CACHE STRING "Set ON/TRUE for building Python Bindings")
set(BUILD_QHULL ON CACHE BOOL "Set ON/TRUE to build QHull")
set(BUILD_FOR_MUJOCO ON CACHE BOOL "Set ON/TRUE for building using MuJoCo")

add_definitions(-Og -fPIC -g -Werror )
if(APPLE)
	set(CMAKE_CXX_STANDARD 17)
else()
	set(CMAKE_CXX_STANDARD 17)
	add_definitions(-lstdc++fs )
	# add_definitions(-lstdc++fs)
	link_libraries(stdc++fs)
	# add_definitions(-lc++fs)
endif()
find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
# link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})
link_libraries(
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	# ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
	)

find_package(Eigen3 CONFIG REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Eigen version: ${Eigen3_VERSION} ")

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

if(BUILD_FOR_TORCH)
	if(DEFINED ENV{Torch_DIR})
		message(STATUS "Building with Torch")
  	find_package(Torch REQUIRED)
  	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
  else()
		message(STATUS "NOT building with Torch")
		message("Please download libtorch C++, place in external folder and setup the enviornmental variable Torch_DIR accordingly.")
    add_definitions(-DTORCH_NOT_BUILT)
  endif()
else()
	message(STATUS "NOT building with Torch")
  add_definitions(-DTORCH_NOT_BUILT)
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
if(BUILD_FOR_MUJOCO)
    message("Building with mujoco.")

	include_directories(src/prx/mujoco)
	add_subdirectory(src/prx/mujoco)
else()
    message("Not building with mujoco.")
    add_definitions(-DMUJOCO_NOT_BUILT)
endif()

# TODO: Make a shared libraries.
add_subdirectory(src/prx/planning)
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/visualization)
add_subdirectory(src/prx/factor_graphs)

if (BUILD_FOR_PYTHON)
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


# Add the "out" directory: "PATH/TO/ML4KP/out"
file(MAKE_DIRECTORY out)

# include(ExternalProject)
# set(QHULL_SRC_DIR ${CMAKE_BINARY_DIR}/third_party/qhull)
# set(QHULL_BIN_DIR ${CMAKE_BINARY_DIR}/third_party/qhull/build)
# ExternalProject_Add(qhull
#     GIT_REPOSITORY https://github.com/qhull/qhull.git
#     GIT_TAG v8.0.2
#     GIT_PROGRESS TRUE
#     SOURCE_DIR ${QHULL_SRC_DIR}
#     BINARY_DIR ${QHULL_BIN_DIR}
#     CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
#     CMAKE_CACHE_ARGS ""
#     # BUILD_COMMAND "${MAKE_COMMAND}  "
#     INSTALL_COMMAND ""
#     # TEST_COMMAND "${CMAKE_COMMAND} test "
# )
