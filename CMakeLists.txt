cmake_minimum_required(VERSION 3.5)
project(ML4KP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

cmake_policy(SET CMP0079 NEW)

set(BUILD_FOR_PYTHON OFF CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_TORCH  ON CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_FOR_MUJOCO ON CACHE BOOL "Set ON/TRUE for building using MuJoCo")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")

set(${PROJECT_NAME}_VERSION 1.0)
set(${PROJECT_NAME}_HEADERS "" CACHE STRING "Headers to be exported")
set(${PROJECT_NAME}_LIBRARIES "" CACHE STRING "Libraries to be exported")

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

# TODO: Make a shared libraries.
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/planning)
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
  "${CMAKE_CURRENT_BINARY_DIR}/ClimbingStats/ClimbingStatsConfigVersion.cmake"
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