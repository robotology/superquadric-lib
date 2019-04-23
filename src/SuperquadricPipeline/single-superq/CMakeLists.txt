#Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#Author: Giulia Vezzani <giulia.vezzani@iit.it>

#This library is free software; you can redistribute it and/or
#modify it under the terms of the GNU Lesser General Public
#License as published by the Free Software Foundation; either
#version 2.1 of the License, or (at your option) any later version.

#This library is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public
#License along with this library; if not, write to the Free Software
#Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

set(EXE_TARGET_NAME Superquadric-Pipeline-Single)

set(${EXE_TARGET_NAME}_SRC
        main.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME} SuperquadricLibModel SuperquadricLibVis SuperquadricLibGrasp)


install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
