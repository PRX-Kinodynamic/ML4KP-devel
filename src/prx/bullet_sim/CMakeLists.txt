## Bullet_sim submodule of DirtMP

# TODO: Use submodule? if we are using a fork might as well integratated it here...
# Right now it has to be done manually, the following code needs to be 
# corrected to use the bullet submodule.
option(USE_BULLET_SUBMODULE "Check submodules during build" OFF)
if(APPLE OR USE_BULLET_SUBMODULE)
	# According to this thread: https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=9&t=9443&p=31841&hilit=link+mac#p31841
	# is bad idea to link bullet... aka don't link a linkable library 
	# instead add bullet as part of your sources
	# because why use Cmake and linking capabilities?
	if (USE_BULLET_SUBMODULE)
		find_package(Git QUIET)
		if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
		# Update submodules as needed
		    option(GIT_SUBMODULE "Check submodules during build" ON)
		    if(GIT_SUBMODULE)
		        message(STATUS "Submodule update")
		        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
		                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
		        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
		            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
		        endif()
		    endif()
		endif()
		if(NOT EXISTS "${PROJECT_SOURCE_DIR}/src/prx/bullet_sim/external/bullet3/CMakeLists.txt")
	    	message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
		endif()
	endif (USE_BULLET_SUBMODULE)

	add_definitions(
		-DUSE_DOUBLE_PRECISION
		-DBUILD_CLSOCKET
		)
	# set(BUILD_PYBULLET OFF CACHE BOOL)
	# set(BUILD_EGL OFF CACHE BOOL)
	set_property(SOURCE "external/bullet3" PROPERTY BUILD_PYBULLET OFF)
	set_property(SOURCE "external/bullet3" PROPERTY BUILD_EGL OFF)

	# SUBDIRS(external/bullet3)
	
	add_subdirectory(external/bullet3/)
	# set(BULLET_PHYSICS_SOURCE_DIR "/Users/Gary/pracsys/bullet3/")

	# target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC
	set(BULLET_INCLUDE_DIR 
		external/bullet3/src
		external/bullet3/examples
		external/bullet3/examples/SharedMemory
		external/bullet3/examples/RobotSimulator
		external/bullet3/examples/ThirdPartyLibs
		external/bullet3/examples/MultiThreading/b3PosixThreadSupport.cpp
		external/bullet3/examples/MultiThreading/b3PosixThreadSupport.h
		external/bullet3/examples/ThirdPartyLibs/enet/include
		external/bullet3/examples/ThirdPartyLibs/clsocket/src
		CACHE PATH "Bullet include directory"	
	)
	message(STATUS "Setting BULLET_PHYSICS_PATH to: ${CMAKE_CURRENT_SOURCE_DIR}/external/bullet3")
	set(BPP "${CMAKE_CURRENT_SOURCE_DIR}/external/bullet3")
elseif (DEFINED ENV{BULLET_PHYSICS_PATH})
	# add_definitions(-DBULLET_PHYSICS_PATH="$ENV{BULLET_PHYSICS_PATH}")
	set(BPP "$ENV{BULLET_PHYSICS_PATH}")
	message("Using custom Bullet")
	set(BULLET_INCLUDE_DIR "$ENV{BULLET_PHYSICS_PATH}/include/bullet" CACHE PATH "Bullet include directory")
	set(BULLET_LIBRARY_DIR "$ENV{BULLET_PHYSICS_PATH}/lib" CACHE PATH "Bullet lib directory")

	find_package (Threads REQUIRED)
	LINK_LIBRARIES( pthread )
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	#set(LIBS ${LIBS} m rt pthread)
	message(STATUS "Bullet include directory: ${BULLET_INCLUDE_DIR}")
	message(STATUS "Bullet library directory: ${BULLET_LIBRARY_DIR}")
else()
	message("[${CMAKE_PROJECT_NAME}] Bullet NOT FOUND and 'BULLET_PHYSICS_PATH' not set. See README on how to properly setup Bullet.")
	return()
endif()
# # endif()
# add_subdirectory(../external/bullet3)

file(GLOB_RECURSE SRC_PRX_LIB_BULLET_SIM  
	collision_checking/*.cpp
	external/b3RobotSimulatorClientAPI.cpp
	loaders/*.cpp
	plants/*.cpp
	bullet_simulator.cpp
	)

add_library(${PROJECT_NAME}_bullet_sim STATIC ${SRC_PRX_LIB_BULLET_SIM})

target_compile_definitions(${PROJECT_NAME}_bullet_sim PUBLIC -DBULLET_PHYSICS_PATH="${BPP}")
# target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC "/Users/Gary/pracsys/bullet3/examples/SharedMemory/")
target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC ".")
target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC ${BULLET_INCLUDE_DIR})




if(APPLE)
	FIND_LIBRARY(COCOA NAMES Cocoa)
	# FIND_LIBRARY(COCOA_LIBRARY Cocoa)
	FIND_PACKAGE(OpenGL REQUIRED)
	find_package(GLUT REQUIRED)

	target_include_directories(${PROJECT_NAME}_bullet_sim INTERFACE ${OPENGL_INCLUDE_DIR})
	target_include_directories(${PROJECT_NAME}_bullet_sim INTERFACE ${GLUT_INCLUDE_DIR})
	target_link_directories(${PROJECT_NAME}_bullet_sim INTERFACE ${GLUT_LIBRARY_DIR})
    # include_directories(${GLUT_INCLUDE_DIR})
	set(CMAKE_MACOSX_RPATH 0)
    	SET_TARGET_PROPERTIES(${PROJECT_NAME}_bullet_sim  PROPERTIES FRAMEWORK true)

else()
        ADD_DEFINITIONS(-DBT_USE_EGL)
	ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
	# ADD_DEFINITIONS("-DGLEW_STATIC")
	ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

endif()

# # BulletSoftBody -> BulletDynamics -> BulletCollision -> LinearMath).
# target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC
# 	BulletRoboticsGUI BulletRobotics BulletExampleBrowserLib  
# 	BulletFileLoader BulletWorldImporter BulletSoftBody
# 	BulletDynamics BulletCollision BulletInverseDynamicsUtils 
# 	BulletInverseDynamics LinearMath OpenGLWindow    
# 	gwen 
# 	BussIK
# 	Bullet3Common 
# 	clsocket
# 	Bullet2FileLoader
# 	Bullet3Collision
# 	Bullet3Dynamics
# 	BulletXmlWorldImporter
# 	Bullet3Geometry
# 	ConvexDecomposition
# 	Bullet3OpenCL_clew
# 	GIMPACTUtils
# 	HACD
# 	)


if(APPLE)

# 	#TODO: check for other systems
	target_link_libraries(${PROJECT_NAME}_bullet_sim INTERFACE
		# BulletRoboticsGUI BulletExampleBrowserLib BulletRobotics BulletFileLoader
		# BulletWorldImporter BulletInverseDynamicsUtils  
		OpenGLWindow gwen BussIK
		pthread dl
		${OPENGL_LIBRARIES} 
		${GLUT_LIBRARIES}
		${COCOA}
		${OPENGL_gl_LIBRARY}
		${OPENGL_glu_LIBRARY}
		)
#         # LINK_LIBRARIES(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

else()
	
	target_link_directories(${PROJECT_NAME}_bullet_sim PUBLIC
	 	${BULLET_LIBRARY_DIR}
	 	)
	target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC
    		${CMAKE_THREAD_LIBS_INIT}
    		rt
		pthread 
		dl
		gwen
		${DL} 
		)
endif()

target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC ${CMAKE_DL_LIBS})
target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC pthread
	BulletRoboticsGUI BulletRobotics BulletExampleBrowserLib  
	BulletFileLoader BulletWorldImporter BulletSoftBody
	BulletDynamics BulletCollision BulletInverseDynamicsUtils 
	BulletInverseDynamics LinearMath OpenGLWindow    
	gwen 
	BussIK
	Bullet3Common 
	clsocket
	)