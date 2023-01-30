cmake_minimum_required(VERSION 3.0)
project(parking_slam)

#****************************************************************#
###################################
##            cmake              ##
###################################
include($ENV{DEVOPS_ROOT_DIR}/cmake/thirdparty.cmake)


###################################
##      extra dependency         ##
###################################

option(USE_HIOS "option for hios" ON)
if (USE_HIOS)
    add_definitions(-DUSE_HIOS)
endif()

if (EXISTS /build_env/es10)		
add_definitions(-DVES10)
endif()

if (EXISTS /build_env/es8)		
add_definitions(-DVES8)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
   set(PKGS common map_engine)
else()
set(PKGS common error_relation rte map_engine)
endif()

set(PKG_LIBRARIES)
foreach(pkg ${PKGS})
   find_package(${pkg})
   include_directories(${${pkg}_INCLUDE_DIRS})
   link_directories(${${pkg}_LIBRARY_DIRS})
   message(STATUS "${pkg}_INCLUDE_DIRS ${${pkg}_INCLUDE_DIRS}")
   message(STATUS "${pkg}_LIBRARY_DIRS ${${pkg}_LIBRARY_DIRS}")
   list(APPEND PKG_LIBRARIES ${${pkg}_LIBRARIES})
endforeach()

find_package(catkin REQUIRED)
# find_package(catkin REQUIRED COMPONENTS xxx)

catkin_package(
   # INCLUDE_DIRS include
   # LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS other_catkin_pkg
#    DEPENDS system_lib
)

#****************************************************************#
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
# include
    algorithm/include
    thirdparty/GeographicLib/include
    thirdparty/fast_gicp/include
)

# file(GLOB PROTO_FILES
#    proto/*.proto
#    proto/**/*.proto
# )
# PROTOBUF_GENERATE_CPP(${proto_gen_folder} PROTO_SRCS PROTO_HDRS "${PROTO_FILES}")

file(GLOB PROTO_FILES
    proto/*.proto
    proto/**/*.proto
)
PROTOBUF_GENERATE_CPP(${proto_gen_folder} PROTO_SRCS PROTO_HDRS "${PROTO_FILES}")
add_library(parking_slam_proto SHARED ${PROTO_SRCS} ${VERS_SRCS})
target_link_libraries(
   parking_slam_proto
    ${PKG_LIBRARIES}
    ${HIOS_ALL_LIBRARIES}
    ${catkin_LIBRARIES}
)

file(GLOB_RECURSE SRCS 
     algorithm/*.cpp 
     algorithm/common/*.cpp 
     algorithm/common/msg/*.cpp
     algorithm/registration/icp/*.cpp 
     algorithm/registration/voxel/*.cpp 
     algorithm/factors/*.cpp
     algorithm/math/*.cpp
     data_center/*.cpp 
     exec/*.cpp 
     )
# file(GLOB_RECURSE SRCS *.cpp)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
   include_directories(/build_env/x86/thirdparty/pangolin/include)
   link_directories(/build_env/x86/thirdparty/pangolin/lib)
   set(LIBS
      ${HIOS_ALL_LIBRARIES}
      ${PKG_LIBRARIES}
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${Pangolin_LIBRARIES}
      ${GTSAM_LIBRARIES}
      pangolin 
      GL 
      GLEW
      libGeographiccc
      parking_slam_proto)
else()
list(FILTER SRCS EXCLUDE REGEX ".*exec/visualizer_exec.cpp$")
list(FILTER SRCS EXCLUDE REGEX ".*visualizer.cpp$")
   set(LIBS
      ${HIOS_ALL_LIBRARIES}
      ${PKG_LIBRARIES}
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${GFLAGS_LIBRARIES}
      ${PROJ_LIBRARIES} 
      ${TINYXML2_LIBRARIES} 
      ${ABSEIL_LIBRARIES}
      ${JPEG_LIBRARIES}
      ${PCL_LIBRARIES}
      ${GTSAM_LIBRARIES}
      libGeographiccc
      parking_slam_proto)
endif()

add_subdirectory(thirdparty/GeographicLib)

# add .version segment to libs
MARK_VERSION(VERS_SRCS VERS_HDRS)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED ${SRCS} ${VERS_SRCS} ${VERS_HDRS})
# add_dependencies(${PROJECT_NAME} xxx)
target_link_libraries(${PROJECT_NAME} ${LIBS})

## Declare a C++ executable
# set(target xxx)
# add_executable(${target} xxx.cpp)
# add_dependencies(${target} ${PROJECT_NAME})
# target_link_libraries(${target} ${PROJECT_NAME} ${LIBS})


##########
## test ##
##########
# if (${BUILD_TEST})
#    add_subdirectory(test)
#    add_subdirectory(ut)
# endif()

#############
## Install ##
#############
# set_property(TARGET ${PROJECT_NAME}
#    PROPERTY VERSION $ENV{MODULE_VERSION})

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS parking_slam_proto
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

   add_executable(test_lio_mapping test/test_lio_mapping.cpp)
   target_link_libraries(test_lio_mapping ${LIBS} ${PROJECT_NAME})

   add_executable(test_lio_localization test/test_lio_localization.cpp)
   target_link_libraries(test_lio_localization ${LIBS} ${PROJECT_NAME})

   add_executable(test_generate_config test/test_generate_config.cpp)
   target_link_libraries(test_generate_config ${LIBS} ${PROJECT_NAME})

   add_executable(test_load_map test/test_load_map.cpp)
   target_link_libraries(test_load_map ${LIBS} ${PROJECT_NAME})

   add_executable(test_rpc test/test_rpc.cpp)
   target_link_libraries(test_rpc ${LIBS} ${PROJECT_NAME})

   install(TARGETS test_lio_mapping
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   install(TARGETS test_lio_localization
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

   install(TARGETS test_generate_config
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

   install(TARGETS test_load_map
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

   install(TARGETS test_rpc
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

   install(DIRECTORY
      config/arch_x86/
      DESTINATION ${CATKIN_PACKAGE_CONFIG_DESTINATION}
   )
else()
   install(DIRECTORY
      config/qnx/
      DESTINATION ${CATKIN_PACKAGE_CONFIG_DESTINATION}
   )
endif()

## Mark cpp header files for installation
# install(DIRECTORY ./
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN "*.hpp"
#    PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY
#    config/
#    DESTINATION ${CATKIN_PACKAGE_CONFIG_DESTINATION}
#)

# deb_package(
#    VERSION $ENV{MODULE_VERSION}
#    DESTINATION $ENV{DEB_INSTALL_PREFIX}
# )
