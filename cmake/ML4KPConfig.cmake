
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ML4KPConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

#include("/usr/local/lib/.cmake")

find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)
find_package(Threads REQUIRED)

if(OFF)
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



foreach(LIBRARY ;External;Utilities;Simulation;Planning;Visualization)
	include("${PACKAGE_PREFIX_DIR}/lib/cmake/ML4KP/${LIBRARY}Targets.cmake")
endforeach()

include("${PACKAGE_PREFIX_DIR}/lib/cmake/ML4KP/ML4KPTargets.cmake")

set_and_check(ML4KP_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set_and_check(ML4KP_LIBRARY_DIR "${PACKAGE_PREFIX_DIR}/lib")

check_required_components(ML4KP)

find_package(YAMLCPP NAMES Yaml-cpp YamlCpp)
