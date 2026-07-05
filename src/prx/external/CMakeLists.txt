## External libraries of DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")
include(FetchContent)

file(GLOB_RECURSE HEADER_PRX_LIB_EXTERNAL_PQP RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp ./*.h)
file(GLOB_RECURSE SRC_PRX_LIB_EXTERNAL_PQP  PQP/*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_EXTERNAL_PQP}" PARENT_SCOPE)

find_package(YAMLCPP NAMES Yaml-cpp YamlCpp yaml-cpp)
message(STATUS "yaml: ${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR}")
include_directories(${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})



message(STATUS "foxglove:")
if (BUILD_WITH_FOXGLOVE)	
	if (APPLE)	
		set(foxglove_url "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.25.1/foxglove-v0.25.1-cpp-x86_64-apple-darwin.zip")
		set(foxglove_hash "3531502efd958be3f9dca06a099c167512dd3006b98ac3eecf8250d376eaa16b")
		message(STATUS "foxglove Apple")
	elseif(LINUX)
		set(foxglove_url "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.25.1/foxglove-v0.25.1-cpp-x86_64-unknown-linux-gnu.zip")
		set(foxglove_hash "0b9d348df3a8e98d2c2cee2360cc9f52e83bb3445044debd771d1b46dc358be4")
		message(STATUS "foxglove Apple")
    	add_definitions(-DFOXGLOVE_BUILT)
	else()
    	add_definitions(-DFOXGLOVE_NOT_BUILT)
		set(BUILD_WITH_FOXGLOVE OFF)
		set(FOXGLOVE_SDK_CPP_SOURCES "")
	endif()
endif()


if (BUILD_WITH_FOXGLOVE)	
	FetchContent_Declare(
    	foxglove_sdk
    	DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    	# TODO: Copy the URL and SHA for the appropriate .zip file from here:
    	# https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true
    	URL ${foxglove_url}
    	URL_HASH SHA256=${foxglove_hash}
	)
	FetchContent_MakeAvailable(foxglove_sdk)

	# Load the SDK's CMake package config (shipped under lib/cmake/foxglove-sdk/).
	# It defines IMPORTED targets for the prebuilt C libraries and the
	# foxglove_sdk_add_cpp_library() helper used below.
	message("foxglove_sdk_SOURCE_DIR: ${foxglove_sdk_SOURCE_DIR}")
	find_package(foxglove-sdk CONFIG REQUIRED HINTS "${foxglove_sdk_SOURCE_DIR}")
	# Compile the SDK's C++ wrapper sources against our toolchain.
			
	set(FOXGLOVE_CMAKE_DIR "${foxglove_sdk_SOURCE_DIR}/lib/cmake/foxglove-sdk/")
	get_filename_component(prx_foxglove_prefix "${FOXGLOVE_CMAKE_DIR}/../../.." ABSOLUTE)
	
	set(prx_foxglove_lib_dir "${prx_foxglove_prefix}/lib")
	set(prx_foxglove_include_dir "${prx_foxglove_prefix}/include")
	set(prx_foxglove_src_dir "${prx_foxglove_prefix}/src")

	include("${FOXGLOVE_CMAKE_DIR}/foxglove-c-imports.cmake")
	foxglove_sdk_import_c_libs(
	  STATIC_LIB "${_foxglove_lib_dir}/${CMAKE_STATIC_LIBRARY_PREFIX}foxglove${CMAKE_STATIC_LIBRARY_SUFFIX}"
	  SHARED_LIB "${_foxglove_lib_dir}/${CMAKE_SHARED_LIBRARY_PREFIX}foxglove${CMAKE_SHARED_LIBRARY_SUFFIX}"
	  SHARED_IMPLIB "${_foxglove_lib_dir}/foxglove.dll.lib"
	)

	include("${FOXGLOVE_CMAKE_DIR}/foxglove-sources.cmake")
	set(FOXGLOVE_SDK_CPP_SOURCES
  		${FOXGLOVE_CPP_HANDWRITTEN_SOURCES}
  		${FOXGLOVE_CPP_GENERATED_SOURCES}
	)
	list(TRANSFORM FOXGLOVE_SDK_CPP_SOURCES PREPEND "${prx_foxglove_src_dir}/")

endif()




if (${BUILD_STATIC_LIBRARY})
	add_library(External STATIC ${SRC_PRX_LIB_EXTERNAL_PQP} ${FOXGLOVE_SDK_CPP_SOURCES}) 
else()
	add_library(External ${SRC_PRX_LIB_EXTERNAL_PQP} ) 
endif()

if(TARGET yaml-cpp )
	target_link_libraries(External PUBLIC yaml-cpp )
elseif(TARGET yaml-cpp::yaml-cpp )
	target_link_libraries(External PUBLIC yaml-cpp::yaml-cpp)
endif()

if (BUILD_WITH_FOXGLOVE)	
    target_link_libraries(External PUBLIC foxglove-static)
  	target_include_directories(External PUBLIC
								"$<BUILD_INTERFACE:${prx_foxglove_include_dir}>"
								)
  	 # "${prx_foxglove_include_dir}")

endif()


if(BUILD_FOR_GTSAM)
	get_target_property(GTSAM_INCLUDE_DIRECTORIES gtsam INTERFACE_INCLUDE_DIRECTORIES)
	message(STATUS "GTSAM_INCLUDE_DIRECTORIES ${GTSAM_INCLUDE_DIRECTORIES}")
	target_include_directories(External INTERFACE
							"$<BUILD_INTERFACE:${GTSAM_INCLUDE_DIRECTORIES}>"
                            "$<INSTALL_INTERFACE:${GTSAM_INCLUDE_DIRECTORIES}>"
														)
	target_link_libraries(External INTERFACE gtsam)
endif(BUILD_FOR_GTSAM)



target_include_directories(External PUBLIC
							"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

# Silencing all warning for the external (non-prx) code
set_target_properties(External PROPERTIES COMPILE_FLAGS "-w")

if(BUILD_FOR_TORCH AND DEFINED ENV{Torch_DIR})
	message(STATUS "Linking Torch: ${TORCH_LIBRARIES}")
	target_link_libraries(External PUBLIC ${TORCH_LIBRARIES})
endif()
target_link_libraries(External PUBLIC Boost::filesystem Boost::system Boost::serialization Threads::Threads)

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};External;" PARENT_SCOPE)


########################################################
# FetchContent_MakeAvailable(foxglove)

# Assuming the project consists of a single source file, main.cpp
# add_executable(my_program main.cpp)

# Add include directory for Foxglove SDK
# target_include_directories(my_program PRIVATE ${foxglove_SOURCE_DIR}/include)
# target_include_directories(my_program PRIVATE ${foxglove_SOURCE_DIR}/include/foxglove)

# Find all Foxglove SDK source files
# file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
#     "${foxglove_SOURCE_DIR}/src/*.cpp"
#     "${foxglove_SOURCE_DIR}/src/server/*.cpp"
# )

# Add Foxglove SDK source files
# target_sources(my_program PRIVATE ${FOXGLOVE_SOURCES})

# Link against libfoxglove.a
# target_link_libraries(my_program PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a)