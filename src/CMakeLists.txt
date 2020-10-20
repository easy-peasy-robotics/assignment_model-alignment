################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

find_package(YARP REQUIRED COMPONENTS os dev sig math pcl)
find_package(ICUB REQUIRED COMPONENTS iKin)
find_package(VTK REQUIRED)
message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  include(${VTK_USE_FILE})
endif()
find_package(Gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

yarp_add_idl(IDL_GEN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.thrift)
add_executable(assignment_${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${IDL_GEN_FILES})
target_include_directories(assignment_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${GAZEBO_INCLUDE_DIRS})
target_compile_definitions(assignment_${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(assignment_${PROJECT_NAME} PRIVATE ${YARP_LIBRARIES} ICUB::iKin ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${GAZEBO_LIBRARIES})

if (VTK_VERSION VERSION_GREATER_EQUAL "8.90.0")
  vtk_module_autoinit(TARGETS assignment_${PROJECT_NAME} MODULES ${VTK_LIBRARIES})
endif()

install(TARGETS assignment_${PROJECT_NAME} DESTINATION bin)
