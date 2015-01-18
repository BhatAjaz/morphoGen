# Copyright: (C) 2013 RBCS - Robotics, Brain and Cognitive Science
# Authors: Ajaz Ahmad Bhat
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(appname perceptionActionCycleApp)
#icub_app(${appname})

find_path(MODULES_KCL  src/Common/PortMap.h HINTS "${DARWIN_COMMON}/../modules/kcl")

set(kclconfig
   ${MODULES_KCL}/app/ObserverStubApp/conf/config-common.ini
   ${MODULES_KCL}/app/ObserverStubApp/conf/effectorports.ini
   ${MODULES_KCL}/app/ObserverStubApp/conf/affordanceports.ini
   ${MODULES_KCL}/app/ObserverStubApp/conf/attentionports.ini
)

file(GLOB conf 
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini 
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt 
   ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.m
)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

# Removed the installation in $ICUB_ROOT/app 11/10/13
#icub_app_install(${appname} 
#                 FILES ${conf}
#                 DESTINATION conf)
#
#icub_app_install(${appname} 
#                FILES ${scripts}
#                DESTINATION scripts)

yarp_install(FILES ${kclconfig} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/kclconfig)

yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})