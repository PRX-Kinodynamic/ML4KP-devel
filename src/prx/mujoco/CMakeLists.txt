file(GLOB_RECURSE SRC_PRX_LIB_MJSIM
    mj_utils.cpp
    mj_plant.cpp
    mj_simulator.cpp
)

add_library(${PROJECT_NAME}_mjsim STATIC ${SRC_PRX_LIB_MJSIM})

target_compile_definitions(${PROJECT_NAME}_mjsim PUBLIC -DMJ_PATH="$ENV{MJ_PATH}")
target_include_directories(${PROJECT_NAME}_mjsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME}_mjsim PUBLIC ${MJ_INCLUDE_DIR})

if(APPLE)
    FIND_LIBRARY(COCOA NAMES Cocoa)
    find_package(glfw3 REQUIRED)
    target_include_directories(${PROJECT_NAME}_mjsim INTERFACE ${GLFW_INCLUDE_DIRS})
    target_include_directories(${PROJECT_NAME}_mjsim INTERFACE ${MJ_INCLUDE_DIR})
    target_link_directories(${PROJECT_NAME}_mjsim INTERFACE ${MJ_LIB_DIR})
    target_link_libraries(${PROJECT_NAME}_mjsim INTERFACE
        ${MJ_LIB_DIR}/libmujoco.2.3.0.dylib
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
