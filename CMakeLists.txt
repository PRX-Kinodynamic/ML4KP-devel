cmake_minimum_required(VERSION 3.5)
project(ML4KP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(BUILD_SHARED_LIBS True)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
cmake_policy(SET CMP0079 NEW)

set(BUILD_FOR_PYTHON OFF CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_TORCH  ON CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")
set(BUILD_FOR_GAZEBO OFF  CACHE BOOL "SET ON/TRUE for building for Gazebo")
set(BUILD_FOR_FACTOR_GRAPHS ON CACHE BOOL "Set ON/TRUE for building tests")
set(BUILD_FOR_MUJOCO ON CACHE BOOL "Set ON/TRUE for building using MuJoCo")

set(${PROJECT_NAME}_VERSION 1.0)
set(${PROJECT_NAME}_HEADERS "" CACHE STRING "Headers to be exported")
set(${PROJECT_NAME}_LIBRARIES "" CACHE STRING "Libraries to be exported")
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(${PROJECT_NAME}_VERSION 1.0)
set(${PROJECT_NAME}_HEADERS "" CACHE STRING "Headers to be exported")
set(${PROJECT_NAME}_LIBRARIES "" CACHE STRING "Libraries to be exported")

if( NOT CMAKE_BUILD_TYPE )
	set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build types" FORCE)
endif()

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
	)

find_package(Eigen3 3.4 CONFIG REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Eigen version: ${Eigen3_VERSION} ")

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

if(BUILD_FOR_TORCH)
	if(DEFINED ENV{Torch_DIR})
        find_package(Torch REQUIRED)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
		# list(APPEND ${PROJECT_NAME}_LIBRARIES "${TORCH_LIBRARIES}")
    else()
		message("Please download libtorch C++, place in external folder and setup the enviornmental variable Torch_DIR accordingly.")
        add_definitions(TORCH_NOT_BUILT)
    endif()
else()
    add_definitions(-DTORCH_NOT_BUILT)
endif()



include_directories(src/prx/external)
include_directories(src/prx/visualization)
include_directories(src/prx/simulation)
include_directories(src/prx/planning)
include_directories(src/prx/utilities)

if(BUILD_FOR_MUJOCO)
    message("Building with mujoco.")

	include_directories(src/prx/mujoco)
	add_subdirectory(src/prx/mujoco)
else()
    message("Not building with mujoco.")
    add_definitions(-DMUJOCO_NOT_BUILT)
endif()

# TODO: Make a shared libraries.
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/planning)
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


# Install the library
include(CMakePackageConfigHelpers)
message(STATUS "Installing ${PROJECT_NAME} in: ${CMAKE_INSTALL_PREFIX}")
# SET(LIB_DESTINATION "lib" CACHE STRING "Library directory name")
set(LIB_INSTALL_DIR "lib/" CACHE STRING "The subdirectory to the libraries")
set(INCLUDE_INSTALL_DIR "include/" CACHE STRING "The subdirectory to the header prefix")

foreach(HEADER IN ITEMS ${${PROJECT_NAME}_HEADERS})
	get_filename_component(H_DIR ${HEADER} DIRECTORY )
	install(FILES src/${HEADER} DESTINATION ${INCLUDE_INSTALL_DIR}/${H_DIR})
endforeach()

install(TARGETS ${${PROJECT_NAME}_LIBRARIES} EXPORT ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_DIR})

configure_package_config_file(cmake/ML4KPConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/ML4KPConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cmake/${PROJECT_NAME}
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ML4KPConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/ML4KPConfig.cmake
              # ${CMAKE_CURRENT_BINARY_DIR}/cmake/ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/share/cmake )
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/DirtMPConfig.cmake.in 
# 	${CMAKE_CURRENT_SOURCE_DIR}/DirtMPConfig.cmake
# 	@ONLY ESCAPE_QUOTES
# 	)
# # install(TARGETS myexe EXPORT myproj DESTINATION bin)
# install(FILES DirtMPConfig.cmake DESTINATION ${LIB_DESTINATION})
# install(EXPORT DirtMP DESTINATION ${LIB_DESTINATION})
