# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(Python_ADDITIONAL_VERSIONS "3.6" CACHE STRING "Python to use for binding")
find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtksys vtkRenderingCore NO_MODULE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(${CMAKE_VERSION} VERSION_GREATER 3.13)
    cmake_policy(SET CMP0078 NEW)
endif()

if(${CMAKE_VERSION} VERSION_GREATER 3.14)
    cmake_policy(SET CMP0086 NEW)
endif()

find_package(SWIG REQUIRED)
set(UseSWIG_MODULE_VERSION 2)

include(${SWIG_USE_FILE})

include(${VTK_USE_FILE})

#SWIG_EXPORT_ITERATOR_METHODS
SET(CMAKE_SWIG_FLAGS "")

set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

set(swig_name "superquadric_bindings")
set_source_files_properties(${swig_name}.i PROPERTIES CPLUSPLUS ON)
#set_source_files_properties(${swig_name}.i PROPERTIES SWIG_FLAGS "-includeall")


swig_add_library(${swig_name}
    TYPE SHARED
    LANGUAGE python
    SOURCES ${swig_name}.i)

set_property(SOURCE ${swig_name}.i APPEND PROPERTY SWIG_FLAGS "-py3" "-DSWIGPYTHON3")

set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-modern")

target_link_libraries(${swig_name} PUBLIC
	SuperquadricLibModel
	SuperquadricLibGrasp
	SuperquadricLibVis
	${VTK_LIBRARIES}
    ${PYTHON_LIBRARIES})

set_property(TARGET ${swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

target_include_directories(${swig_name} PRIVATE ${VTK_INCLUDE_DIRS})
target_include_directories(${swig_name} PRIVATE ${PYTHON_INCLUDE_PATH})
if(NOT TARGET Eigen3)
    target_include_directories(${swig_name} PUBLIC ${EIGEN3_INCLUDE_DIR})
	target_link_libraries(${swig_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
else()
    target_link_libraries(${swig_name} PUBLIC Eigen3::Eigen)
endif()

install(
    TARGETS ${swig_name}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/superquadriclib/bindings
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/superquadriclib/bindings
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/superquadriclib/bindings)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${swig_name}.py
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/superquadriclib/bindings)





