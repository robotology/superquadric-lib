set(EXE_TARGET_NAME Superquadric-Pipeline-Yarp)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()


set(${EXE_TARGET_NAME}_SRC
		src/main.cpp
        #src/idl.thrift
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME} ${YARP_LIBRARIES} SuperquadricLibModel SuperquadricLibVis SuperquadricLibGrasp)


install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

#icubcontrib_add_uninstall_target()
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

file(GLOB ini app/conf/*.ini)
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
