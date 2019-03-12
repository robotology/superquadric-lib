
add_subdirectory(SuperquadricModel)

if(SUPERQ_GRASP)
	add_subdirectory(SuperquadricGrasp)
endif()

if(SUPERQ_VIS)
	add_subdirectory(SuperquadricVis)
endif()


