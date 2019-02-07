set(EXE_TARGET_NAME Superquadric-Pipeline-Yarp)

find_package(YARP CONFIG REQUIRED
			COMPONENTS
			OS
			math
			sig
			gsl)

find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})


set(${EXE_TARGET_NAME}_SRC
		src/main.cpp
)

yarp_add_idl(IDL_GEN_FILES src/idl.thrift)
source_group("IDL Files" FILES src/idl.thrift)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} src/idl.thrift ${IDL_GEN_FILES})

target_link_libraries(${EXE_TARGET_NAME}
					  ${YARP_LIBRARIES} 
					  SuperquadricLibModel 
					  SuperquadricLibVis
					  SuperquadricLibGrasp
					  ctrlLib
                      ${ICUB_LIBRARIES}
                      iKin
                      YARP::YARP_OS
                      YARP::YARP_init
                      YARP::YARP_math
                      YARP::YARP_sig
					  YARP::YARP_gsl
					  )


install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

#icubcontrib_add_uninstall_target()
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

file(GLOB ini app/conf/*.ini)
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
