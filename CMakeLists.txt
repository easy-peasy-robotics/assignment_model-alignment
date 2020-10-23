################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

cmake_minimum_required(VERSION 3.12)
project(model-alignment VERSION 1.0.0)

find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# To find Eigen
set(PCL_MODULE_PATH "$ENV{ROBOT_CODE}/pcl/cmake/Modules")
list(APPEND CMAKE_MODULE_PATH ${PCL_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()
add_subdirectory(src)
add_subdirectory(gazebo)
include(AddUninstallTarget)

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME})
