cmake_minimum_required(VERSION 3.0.2)
project(AWSSDK)
set(AWSSDK_VERSION 1.6.53)
set(AWSSDK_MD5 8c04cd7a0670bc4a0cdcbb2f416efa7a)

if(NOT EXTERNAL_INSTALL_LOCATION)
  set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
endif()

#############
## Library ##
#############
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib)

string(REPLACE ";" "$<SEMICOLON>" SERVICES_ARG "${SERVICE}")

set(extra_cmake_args)
if(DEFINED CMAKE_BUILD_TYPE)
  list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
endif()
if(DEFINED CMAKE_TOOLCHAIN_FILE)
  list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  if(ANDROID)
    if(DEFINED ANDROID_ABI)
      list(APPEND extra_cmake_args "-DANDROID_ABI=${ANDROID_ABI}")
    endif()
    if(DEFINED ANDROID_CPP_FEATURES)
      list(APPEND extra_cmake_args "-DANDROID_CPP_FEATURES=${ANDROID_CPP_FEATURES}")
    endif()
    if(DEFINED ANDROID_FUNCTION_LEVEL_LINKING)
      list(APPEND extra_cmake_args "-DANDROID_FUNCTION_LEVEL_LINKING=${ANDROID_FUNCTION_LEVEL_LINKING}")
    endif()
    if(DEFINED ANDROID_NATIVE_API_LEVEL)
      list(APPEND extra_cmake_args "-DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL}")
    endif()
    if(DEFINED ANDROID_NDK)
      list(APPEND extra_cmake_args "-DANDROID_NDK=${ANDROID_NDK}")
    endif()
    if(DEFINED ANDROID_STL)
      list(APPEND extra_cmake_args "-DANDROID_STL=${ANDROID_STL}")
    endif()
    if(DEFINED ANDROID_TOOLCHAIN_NAME)
      list(APPEND extra_cmake_args "-DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME}")
    endif()
  endif()
else()
  list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
endif()
list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

include(ExternalProject)
ExternalProject_Add(AWS_SDK_IMPORT
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/src
  URL https://github.com/aws/aws-sdk-cpp/archive/${AWSSDK_VERSION}.tar.gz
  URL_MD5 ${AWSSDK_MD5}
  TIMEOUT 600
  CMAKE_ARGS
    -DMINIMIZE_SIZE=TRUE
    -DENABLE_TESTING=FALSE
    -DBUILD_ONLY=${SERVICES_ARG}
    -DCPP_STANDARD=11
    -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
    -DTARGET_ARCH=${TARGET_ARCH}
    ${extra_cmake_args}
)

#############
## Install ##
#############

install(DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib/
  DESTINATION lib/
  FILES_MATCHING PATTERN "libaws*.so"
)

install(
  DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include/aws
  DESTINATION include/
)

export(PACKAGE ${PROJECT_NAME})
