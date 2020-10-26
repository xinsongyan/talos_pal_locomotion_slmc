cmake_minimum_required(VERSION 2.8.3)
project(talos_pal_locomotion)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


set(CATKIN_COMPONENTS
    pal_physics_utils
    pal_locomotion
    pal_locomotion_msgs
    terrain_generation
    reflexxes_adapter
    pal_locomotion_dcm_planner
    backward_ros
    kinematic_state_estimator
    pal_locomotion_actions
    inertia_shaping_effort_control)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES tor_pal_locomotion
    CATKIN_DEPENDS ${CATKIN_COMPONENTS}
    #  DEPENDS system_lib
    )

include_directories(
    include
    )

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/wbc_weighted_dynamics_talos.cpp
    src/walking_wbc_base.cpp
    src/walking_wbc_lower_body_torso_head.cpp
    src/walking_wbc_fixed_upper_body.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(execute_motion src/execute_motion.cpp)
target_link_libraries(execute_motion ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_subdirectory(test)

add_executable(safe_controller_initialization src/safe_controller_initialization.cpp)
target_link_libraries(safe_controller_initialization  ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} execute_motion safe_controller_initialization
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(FILES biped_wbc.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
