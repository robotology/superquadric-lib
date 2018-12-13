if(SUPERQ_GRASP)
	add_subdirectory(SuperquadricGrasp)
endif()
if(SUPERQ_MODEL)
	add_subdirectory(SuperquadricModel)
	message(STATUS "${PROJECT_NAME}_TARGETS intermediate: ${${PROJECT_NAME}_TARGETS}")
endif()
if(SUPERQ_VIS)
	add_subdirectory(SuperquadricVis)
endif()

set(${PROJECT_NAME}_TARGETS ${${PROJECT_NAME}_TARGETS} PARENT_SCOPE)

