## External libraries of DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")
include(FetchContent)

file(GLOB_RECURSE HEADER_PRX_LIB_EXTERNAL_PQP RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp ./*.h)
file(GLOB_RECURSE SRC_PRX_LIB_EXTERNAL_PQP  PQP/*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_EXTERNAL_PQP}" PARENT_SCOPE)

find_package(YAMLCPP NAMES Yaml-cpp YamlCpp yaml-cpp)
message(STATUS "yaml: ${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR}")
include_directories(${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})


if (${BUILD_STATIC_LIBRARY})
	add_library(External STATIC ${SRC_PRX_LIB_EXTERNAL_PQP} ) 
else()
	add_library(External ${SRC_PRX_LIB_EXTERNAL_PQP} ) 
endif()

if(TARGET yaml-cpp )
	target_link_libraries(External PUBLIC yaml-cpp )
elseif(TARGET yaml-cpp::yaml-cpp )
	target_link_libraries(External PUBLIC yaml-cpp::yaml-cpp)
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


if (APPLE)	
	FetchContent_Declare(
    foxglove
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    # TODO: Copy the URL and SHA for the appropriate .zip file from here:
    # https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true
    URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.21.0/foxglove-v0.21.0-cpp-x86_64-apple-darwin.zip
    URL_HASH SHA256=08c3b1d495eb607c573203823fc61a6eb35c510903a1c6d25af8e623c754ab70
	)
elseif(LINUX)
	FetchContent_Declare(
    foxglove
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    # TODO: Copy the URL and SHA for the appropriate .zip file from here:
    # https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true
    URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.21.0/foxglove-v0.21.0-cpp-x86_64-unknown-linux-gnu.zip
    URL_HASH SHA256=3073923a62de4b8c99f2abd49ebc8ac218017d09c2e6c9908f991434281c6994
	)
endif()
FetchContent_MakeAvailable(foxglove)
target_include_directories(External PRIVATE ${foxglove_SOURCE_DIR}/include)
target_include_directories(External PRIVATE ${foxglove_SOURCE_DIR}/include/foxglove)
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${foxglove_SOURCE_DIR}/src/*.cpp"
    "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)
target_sources(External PRIVATE ${FOXGLOVE_SOURCES})
target_link_libraries(External PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a)


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

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};External" PARENT_SCOPE)


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