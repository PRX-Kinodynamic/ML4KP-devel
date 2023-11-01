if(BUILD_FOR_MUJOCO)
    find_package( OpenCV REQUIRED )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    if(DEFINED ENV{MJ_PATH})
        message("MuJoCo found at $ENV{MJ_PATH}")
        include_directories($ENV{MJ_PATH}/include)
        link_directories($ENV{MJ_PATH}/build/lib)
        set(MJ_INCLUDE_DIR "$ENV{MJ_PATH}/include/" CACHE PATH "MuJoCo include directory")
        set(MJ_LIB_DIR "$ENV{MJ_PATH}/build/lib" CACHE PATH "MuJoCo library directory")
    else()
        message(FATAL_ERROR "Please download and build MuJoCo and setup the enviornmental variable MJ_PATH accordingly.")
        add_definitions(-DMUJOCO_NOT_BUILT)
    endif()

file(GLOB_RECURSE SRC_PRX_LIB_MJSIM *.cpp)
file(GLOB_RECURSE HEADER_PRX_LIB_MJSIM RELATIVE ${CMAKE_SOURCE_DIR}/src ./*.hpp)

# set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADER_PRX_LIB_MJSIM}" PARENT_SCOPE)

    add_library(${PROJECT_NAME}_mjsim STATIC ${SRC_PRX_LIB_MJSIM})

    target_compile_definitions(${PROJECT_NAME}_mjsim PUBLIC -DMJ_PATH="$ENV{MJ_PATH}")
    target_include_directories(${PROJECT_NAME}_mjsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_include_directories(${PROJECT_NAME}_mjsim PUBLIC ${MJ_INCLUDE_DIR})

    if(APPLE)
        FIND_LIBRARY(COCOA NAMES Cocoa)
        find_package(glfw3 REQUIRED)
        file(GLOB MJ_LIBS ${MJ_LIB_DIR}/*.dylib)
        message(STATUS "Mujoco library: ${MJ_LIBS}")
        message(STATUS "Mujoco include dirs: ${MJ_INCLUDE_DIR}")
        target_include_directories(${PROJECT_NAME}_mjsim PUBLIC ${GLFW_INCLUDE_DIRS})
        target_include_directories(${PROJECT_NAME}_mjsim PUBLIC ${MJ_INCLUDE_DIR})
        target_link_directories(${PROJECT_NAME}_mjsim PUBLIC ${MJ_LIB_DIR})
        target_link_libraries(${PROJECT_NAME}_mjsim PUBLIC
            ${MJ_LIBS}
            glfw
        )
        set(CMAKE_MACOSX_RPATH 0)
        set_target_properties(${PROJECT_NAME}_mjsim PROPERTIES FRAMEWORK true)
    else()
	   find_package(glfw3 REQUIRED)
	   target_include_directories(${PROJECT_NAME}_mjsim INTERFACE ${GLFW_INCLUDE_DIRS})
        target_include_directories(${PROJECT_NAME}_mjsim INTERFACE ${MJ_INCLUDE_DIR})
        target_link_directories(${PROJECT_NAME}_mjsim INTERFACE ${MJ_LIB_DIR})
        target_link_libraries(${PROJECT_NAME}_mjsim INTERFACE
            ${MJ_LIB_DIR}/libmujoco.so
            glfw
        )
        set_target_properties(${PROJECT_NAME}_mjsim PROPERTIES FRAMEWORK true)
    endif()
    # TODO: Take care of this for Linux
    link_directories(${PROJECT_NAME}_mjsim PUBLIC ${MJ_LIB_DIR})
    target_link_libraries(${PROJECT_NAME}_mjsim PUBLIC ${OpenCV_LIBS} )
else()
    message("Not building with mujoco.")
    add_definitions(-DMUJOCO_NOT_BUILT)
endif()
