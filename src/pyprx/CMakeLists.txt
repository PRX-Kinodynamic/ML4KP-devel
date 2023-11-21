## Python bindings for DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")
message("Building python bindings")
string(COMPARE GREATER_EQUAL ${PY_VERSION} "3" USE_PY3)
cmake_policy(SET CMP0148 OLD) 
if (${USE_PY3})
    message("Using Python 3")

    if (${PY_VERSION} EQUAL "3")
        set(Python_versions_nums 3.12 3.11 3.10 3.9 3.8 3.7 3.6 3.5 3.4 3 )
    else()
        set(Python_versions_nums  ${PY_VERSION} )
    endif()

    foreach(PYVER IN LISTS Python_versions_nums)
        find_package(Python  ${PYVER} EXACT COMPONENTS Interpreter Development QUIET)
        find_package(PythonLibs ${PYVER} EXACT QUIET)
        if(${PythonLibs_FOUND})
            message("Found python ${PYVER}")
            string(REPLACE "." "" PYVER_2 ${PYVER})
            find_package(Boost REQUIRED COMPONENTS "python${PYVER_2}")
            message("Boost_PY_LIBS ${Boost_PYTHON${PYVER_2}_LIBRARY}")
            set(Boost_PY_LIBS ${Boost_PYTHON${PYVER_2}_LIBRARY})
            break()
        endif()
    endforeach()
    add_definitions(-DBOOST_PYTHON_STATIC_LIB)
    include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${PYTHON_INCLUDE_DIRS})
else()
    message("Using Python 2.7")
    find_package(Boost REQUIRED COMPONENTS python27)
    set(Python_ADDITIONAL_VERSIONS 2.7)
    find_package(PythonLibs 2.7 REQUIRED)
    set(Boost_PY_LIBS ${Boost_PYTHON27_LIBRARY})
	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${PYTHON_INCLUDE_DIRS})
endif()

file(GLOB_RECURSE SRC_PRX_LIB_PY ./*.cpp)

add_library(Py${PROJECT_NAME} MODULE pyprx.cpp )
set_target_properties(Py${PROJECT_NAME} 
                      PROPERTIES PREFIX ""
                      )
target_include_directories(Py${PROJECT_NAME} PUBLIC ".")

target_link_libraries(Py${PROJECT_NAME} ${Boost_PY_LIBS} ${PYTHON_LIBRARIES})
target_link_libraries(Py${PROJECT_NAME} ${YAMLCPP_LIBRARY} ${YAML_CPP_LIBRARIES})
target_link_libraries(Py${PROJECT_NAME} ${PROJECT_NAME})


