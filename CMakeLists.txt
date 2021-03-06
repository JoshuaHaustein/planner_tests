cmake_minimum_required(VERSION 2.8.3)
project(planner_tests)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        cmake_modules
        roscpp
        sim_env
        box2d_sim_env
        manipulation_planning_suite
        std_msgs
        geometry_msgs
        message_generation
)

## System dependencies are found with CMake's conventions
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED) # TODO: I do not understand why we need this. Shoudn't this be 'inherited' from box2d_sim_env????
include(${QT_USE_FILE}) # TODO: maybe because of this?
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

add_service_files(
        FILES 
        GetActionSpaceInfo.srv
        GetObjectProperties.srv
        SetObjectProperties.srv
        GetState.srv
        Propagate.srv
        SetActiveObjects.srv
        SetState.srv
)

generate_messages (
        DEPENDENCIES 
        std_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planner_tests
  CATKIN_DEPENDS sim_env box2d_sim_env manipulation_planning_suite
  DEPENDS Boost YAML
)

###########
## Build ##
###########
set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)
add_definitions(-Wno-deprecated-register)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_definitions(${QT_DEFINITIONS})

set (MOC_HEADERS
        include/planner_tests/box2d/widget/PushPlannerWidget.h
        include/planner_tests/box2d/widget/SliceWidget.h)
QT4_WRAP_CPP( MOC_SRCS ${MOC_HEADERS})

set(PUSH_PLANNER_SRC
        src/planner_tests/box2d/PushPlanner.cpp
        src/planner_tests/box2d/widget/QtUtils.cpp
        src/planner_tests/box2d/widget/PushPlannerWidget.cpp
        src/planner_tests/box2d/widget/SliceWidget.cpp)

# set (MOC_HEADERS_SORT
#         include/planner_tests/box2d/widget/AlphaSortWidget.h)
# QT4_WRAP_CPP( MOC_SRCS_SORT ${MOC_HEADERS_SORT})

# set(SORT_PLANNER_SRC
#         src/planner_tests/box2d/AlphaSort.cpp
#         src/planner_tests/box2d/widget/QtUtils.cpp
#         src/planner_tests/box2d/widget/AlphaSortWidget.cpp)

set(ORACLE_TRAINING_DATA_SRC
        src/planner_tests/box2d/OracleTrainingDataGen.cpp)

set(SDF_TEST_SRC
        src/planner_tests/box2d/testSDF.cpp)

set(ORACLE_EVALUATION_SRC
        src/planner_tests/box2d/OracleEvaluation.cpp)

set(TRAINING_SERVER_SRC
        src/planner_tests/box2d/OracleTrainingServer.cpp
        src/planner_tests/box2d/training_server/ROSTrainingServer.cpp
        src/planner_tests/box2d/widget/QtUtils.cpp
        src/planner_tests/box2d/widget/PushPlannerWidget.cpp
        src/planner_tests/box2d/widget/SliceWidget.cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

## Declare a C++ library
# add_library(planner_tests
#   src/${PROJECT_NAME}/planner_tests.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(planner_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(box2d_push_planner
         ${PUSH_PLANNER_SRC}
         ${MOC_SRCS}
         )

#  add_executable(box2d_alpha_sort
#          ${SORT_PLANNER_SRC}
#          ${MOC_SRCS_SORT}
#          )

 add_executable(box2d_sdf_test
         ${SDF_TEST_SRC}
         )

add_executable(box2d_oracle_generation
        ${ORACLE_TRAINING_DATA_SRC}
        )

add_executable(box2d_oracle_evaluation
        ${ORACLE_EVALUATION_SRC}
        )

add_executable(box2d_training_server
        ${TRAINING_SERVER_SRC}
        ${MOC_SRCS}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(planner_tests_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(box2d_push_planner
   ${catkin_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
   ${PROTOBUF_LIBRARY}
#   proto
   yaml-cpp
)

# target_link_libraries(box2d_alpha_sort
#    ${catkin_LIBRARIES}
#    ${QT_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${CMAKE_THREAD_LIBS_INIT}
#    ${PROTOBUF_LIBRARY}
#    yaml-cpp
# )

target_link_libraries(box2d_oracle_generation
     ${catkin_LIBRARIES}
     ${Boost_LIBRARIES}
     ${CMAKE_THREAD_LIBS_INIT}
     ${PROTOBUF_LIBRARY}
#     proto
     yaml-cpp
)

target_link_libraries(box2d_sdf_test
     ${catkin_LIBRARIES}
     ${Boost_LIBRARIES}
     ${CMAKE_THREAD_LIBS_INIT}
     ${PROTOBUF_LIBRARY}
     yaml-cpp
)

target_link_libraries(box2d_oracle_evaluation
     ${catkin_LIBRARIES}
     ${Boost_LIBRARIES}
     ${CMAKE_THREAD_LIBS_INIT}
     ${PROTOBUF_LIBRARY}
#     proto
     yaml-cpp
)

target_link_libraries(box2d_training_server
     ${catkin_LIBRARIES}
     ${Boost_LIBRARIES}
     ${CMAKE_THREAD_LIBS_INIT}
     ${PROTOBUF_LIBRARY}
     yaml-cpp
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS planner_tests planner_tests_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_planner_tests.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
