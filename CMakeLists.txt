### Maintainer: flier[@techfak.uni-bielefeld.de] ###
cmake_minimum_required(VERSION 2.8)
PROJECT(pepper_behavior_sandbox)
SET(CMAKE_BUILD_TYPE distribution)

MESSAGE("-- You are building on: ${CMAKE_SYSTEM_NAME}")
MESSAGE("-- If you don't set -DCMAKE_INSTALL_PREFIX, defaults are used.")

SET(PREFIX "/usr/local")
IF (CMAKE_INSTALL_PREFIX)
    SET(PREFIX ${CMAKE_INSTALL_PREFIX})
    MESSAGE("-- Found install PREFIX: " ${PREFIX})
ENDIF()

INSTALL(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/smach/pepper_smach_geniale_2017.py" DESTINATION ${PREFIX}/bin)