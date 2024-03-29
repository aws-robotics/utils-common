# #############
# ## Warning ##
# #############

# # AWS does not provide support for this cloud extension.
# # You are solely responsible for how you configure, deploy, and maintain this
# # cloud extension in your workloads or commercial product(s) or devices.
# # You are solely responsible for testing and configuring the correct AWS SDK version.

# cmake_minimum_required(VERSION 3.0.2)
# project(AWSSDK)
# # User replaces ADD_YOUR_VERSION_HERE and ADD_YOUR_MD5_HERE with their desired AWS SDK version and corresponding MD5
# set(AWSSDK_VERSION ADD_YOUR_VERSION_HERE)
# set(AWSSDK_MD5 ADD_YOUR_MD5_HERE)

# if(NOT EXTERNAL_INSTALL_LOCATION)
#   set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
# endif()

# #############
# ## Library ##
# #############
# file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
# file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib)

# string(REPLACE ";" "$<SEMICOLON>" SERVICES_ARG "${SERVICE}")

# include(ExternalProject)
# ExternalProject_Add(AWS_SDK_IMPORT
#   SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/src
#   URL https://github.com/aws/aws-sdk-cpp/archive/${AWSSDK_VERSION}.tar.gz
#   URL_MD5 ${AWSSDK_MD5}
#   CMAKE_ARGS -DMINIMIZE_SIZE=TRUE -DENABLE_TESTING=FALSE -DBUILD_ONLY=${SERVICES_ARG} -DCPP_STANDARD=11 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
# )

# #############
# ## Install ##
# #############

# install(DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib/
#   DESTINATION lib/
#   FILES_MATCHING PATTERN "libaws*.so"
# )

# install(
#   DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include/aws
#   DESTINATION include/
# )

# export(PACKAGE ${PROJECT_NAME})
