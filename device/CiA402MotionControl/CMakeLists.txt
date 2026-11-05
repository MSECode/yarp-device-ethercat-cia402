# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


add_yarp_device(
    NAME CiA402MotionControl
    TYPE yarp::dev::CiA402MotionControl
    SOURCES CiA402MotionControl.cpp CiA402StateMachine.cpp CstCommandWatchdog.h
    PUBLIC_HEADERS CiA402MotionControl.h CiA402StateMachine.h
    PUBLIC_LINK_LIBRARIES
      YARP::YARP_os
      YARP::YARP_dev
      YarpCiA402::EthercatManager
    CONFIGURE_PACKAGE_NAME CiA402MotionControlPlugins)

if(BUILD_TESTING)
    add_executable(test_cst_command_watchdog tests/test_cst_command_watchdog.cpp)
    target_compile_features(test_cst_command_watchdog PRIVATE cxx_std_17)
    add_test(NAME cia402_cst_command_watchdog COMMAND test_cst_command_watchdog)
    add_executable(test_cst_driver tests/test_cst_driver.cpp CiA402StateMachine.cpp)
    target_compile_features(test_cst_driver PRIVATE cxx_std_17)
    target_link_libraries(test_cst_driver PRIVATE
        YARP::YARP_os YARP::YARP_dev YARP::YARP_init YarpCiA402::EthercatManager)
    add_test(NAME cia402_cst_driver COMMAND test_cst_driver)
endif()
