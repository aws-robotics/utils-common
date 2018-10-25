cmake_minimum_required(VERSION 3.0.2)
project(aws_common)

SET(AWS_COMMON_VERSION 1.0.0)

add_compile_options(-std=c++11)
if (DEFINED ENV{ROS_DISTRO})
    add_definitions(-DCMAKE_ROS_DISTRO="$ENV{ROS_DISTRO}")
endif()
if (DEFINED ENV{ROS_VERSION})
    add_definitions(-DCMAKE_ROS_VERSION="$ENV{ROS_VERSION}")
endif()

if(NOT EXTERNAL_INSTALL_LOCATION)
  set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
endif()

if(EXISTS "${EXTERNAL_INSTALL_LOCATION}/lib/cmake/AWSSDK")
  message(STATUS "Using existing, local AWSSDK installation at ${EXTERNAL_INSTALL_LOCATION}")
  set(AWSSDK_DIR "${EXTERNAL_INSTALL_LOCATION}/lib/cmake/AWSSDK")
  set(AWSSDK_ROOT_DIR ${EXTERNAL_INSTALL_LOCATION})
endif()

find_package(AWSSDK QUIET PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH)
set(SERVICE "core" "logs" "monitoring" "s3" "kinesis" "iot" "lex" "polly")
foreach(_service ${SERVICE})
  list(APPEND ${PROJECT_NAME}_IMPORTED_LIBRARIES "aws-cpp-sdk-${_service}")
endforeach()

if(AWSSDK_FOUND)
  set(SERVICE_NEEDED_FOR_TARGET core)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE_NEEDED_FOR_TARGET AWSSDK_LIBS)
  link_directories("${AWSSDK_LIB_DIR}")
else()
  set(AWSSDK_LIBS ${${PROJECT_NAME}_IMPORTED_LIBRARIES})
endif()

include_directories(include)
## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/sdk_utils/aws_profile_provider.cpp
  src/sdk_utils/client_configuration_provider.cpp
  src/sdk_utils/logging/aws_log_system.cpp
  src/sdk_utils/auth/service_credentials_provider.cpp)

## Add AWS SDK libraries
if(NOT AWSSDK_FOUND)
  add_subdirectory(awssdk)
  add_dependencies(${PROJECT_NAME} AWS_SDK_IMPORT)
endif()

foreach(_imported_lib ${${PROJECT_NAME}_IMPORTED_LIBRARIES})
  if(NOT TARGET ${_imported_lib})
    add_library(${_imported_lib} UNKNOWN IMPORTED)
    set_target_properties(${_imported_lib} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EXTERNAL_INSTALL_LOCATION}/include")
    set_target_properties(${_imported_lib} PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
      IMPORTED_LOCATION "${EXTERNAL_INSTALL_LOCATION}/lib/lib${_imported_lib}.so")
    set_property(TARGET ${_imported_lib} APPEND PROPERTY
      IMPORTED_CONFIGURATIONS RELEASE)
  endif()
endforeach()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${AWSSDK_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE ${AWSSDK_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


#############
### Tests ###
#############
enable_testing()
find_package(GTest QUIET)
if(NOT GTEST_FOUND)
  message(WARNING "Could not find GTest. Not building unit tests.")
else()
  add_executable(test_aws_log_system test/sdk_utils/logging/aws_log_system_test.cpp)
  target_include_directories(test_aws_log_system PRIVATE ${aws_common_INCLUDE_DIRS})
  target_link_libraries(test_aws_log_system
        ${GTEST_LIBRARIES}
        pthread
        ${aws_common_LIBRARIES}
        ${PROJECT_NAME})
  add_test(NAME test_aws_log_system COMMAND test_aws_log_system --gtest_output=xml:test_results/)

  add_executable(test_throttling_manager test/sdk_utils/throttling_manager_test.cpp)
  target_include_directories(test_throttling_manager PRIVATE ${aws_common_INCLUDE_DIRS} ${AWSSDK_INCLUDE_DIR})

  target_link_libraries(test_throttling_manager
        ${GTEST_LIBRARIES}
        pthread
        ${aws_common_LIBRARIES}
        ${PROJECT_NAME})
  add_test(NAME test_throttling_manager COMMAND test_throttling_manager --gtest_output=xml:test_results/)
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}-targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include/
)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(${PROJECT_NAME}ConfigVersion.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(DIRECTORY ${PROJECT_BINARY_DIR}/external/lib/cmake/AWSSDK/ DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)
