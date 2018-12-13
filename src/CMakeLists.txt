add_subdirectory(SuperquadricLib)
if(EXECUTABLE)
	add_subdirectory(SuperquadricPipeline)
endif()

set(${PROJECT_NAME}_TARGETS ${${PROJECT_NAME}_TARGETS} PARENT_SCOPE)
