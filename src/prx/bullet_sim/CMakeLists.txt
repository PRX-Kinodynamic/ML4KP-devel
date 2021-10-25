## Bullet_sim submodule of DirtMP


# find_package(Bullet QUIET)
# print_all_variables()
# message("Found Bullet version: ${Bullet_VERSION}")

# if (Bullet_FOUND)
# 	set(BULLET_PHYSICS_PATH "${Bullet_DIR}")
# 	message("Using system Bullet: ${BULLET_PHYSICS_PATH}")
# else()
	if (DEFINED ENV{BULLET_PHYSICS_PATH})
		add_definitions(-DBULLET_PHYSICS_PATH="$ENV{BULLET_PHYSICS_PATH}")
		message("Using custom Bullet")
		set(BULLET_INCLUDE_DIR "$ENV{BULLET_PHYSICS_PATH}/include/bullet" CACHE PATH "Bullet include directory")
		set(BULLET_LIBRARY_DIR "$ENV{BULLET_PHYSICS_PATH}/lib" CACHE PATH "Bullet lib directory")
	else()
		message("[${CMAKE_PROJECT_NAME}] Bullet NOT FOUND and 'BULLET_PHYSICS_PATH' not set. See README on how to properly setup Bullet.")
		return()
	endif()
# endif()

message("Building with Bullet Physics")
if(APPLE)
	FIND_LIBRARY(COCOA_LIBRARY Cocoa)
	FIND_PACKAGE(OpenGL)
	include_directories(${OPENGL_INCLUDE_DIR})
	find_package(GLUT)
	include_directories(${GLUT_INCLUDE_DIR})
else()
        ADD_DEFINITIONS(-DBT_USE_EGL)
	ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
	ADD_DEFINITIONS("-DGLEW_STATIC")
	ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
		LINK_LIBRARIES(  pthread ${DL} )
endif()
# TODO: There must be a better way of including bullet...
# include_directories("${BULLET_PHYSICS_PATH}/include/bullet")
# include("${PROJECT_SOURCE_DIR}/lib/cmake/bullet/BulletConfig.cmake")

file(GLOB_RECURSE SRC_PRX_LIB_BULLET_SIM  ./*.cpp)

add_library(${PROJECT_NAME}_bullet_sim ${SRC_PRX_LIB_BULLET_SIM})
# message("Including bullet directories: ${BULLET_INCLUDE_DIR}")
include_directories(${PROJECT_NAME}_bullet_sim ${BULLET_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME}_bullet_sim PUBLIC -DBULLET_PHYSICS_PATH="$ENV{BULLET_PHYSICS_PATH}")

target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC ".")
target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC ${BULLET_INCLUDE_DIR})

target_link_directories(${PROJECT_NAME}_bullet_sim PUBLIC
	${BULLET_LIBRARY_DIR}
	)
target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC
	BulletRoboticsGUI BulletExampleBrowserLib BulletRobotics 
	BulletFileLoader BulletWorldImporter BulletSoftBody 
	BulletDynamics BulletCollision BulletInverseDynamicsUtils 
	BulletInverseDynamics LinearMath OpenGLWindow 
	gwen 
	BussIK
	Bullet3Common
	)

target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC pthread ${CMAKE_DL_LIBS})

if(APPLE)
	#TODO: check for other systems
	target_link_libraries(${PROJECT_NAME}_bullet_sim PUBLIC
		# BulletRoboticsGUI BulletExampleBrowserLib BulletRobotics BulletFileLoader
		# BulletWorldImporter BulletInverseDynamicsUtils  
		OpenGLWindow gwen BussIK  
		${OPENGL_LIBRARIES} 
		${GLUT_LIBRARIES}
		)
endif()
	# target_link_libraries(${PROJECT_NAME}     
	# 	pthread ${CMAKE_DL_LIBS})