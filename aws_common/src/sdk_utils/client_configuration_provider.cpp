/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#include <aws/core/Version.h>
#include <aws/core/client/DefaultRetryStrategy.h>
#include <aws/core/platform/OSVersionInfo.h>
#include <aws_common/sdk_utils/aws_profile_provider.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_common/sdk_utils/parameter_reader.h>

#ifndef CLIENT_CONFIG_PREFIX
#define CLIENT_CONFIG_PREFIX "aws_client_configuration"
#endif

namespace Aws {
namespace Client {

bool operator==(const ClientConfiguration & left, const ClientConfiguration & right)
{
  bool result = true;

  result &= (0 == left.region.compare(right.region));
  result &= (0 == left.userAgent.compare(right.userAgent));
  result &= (0 == left.endpointOverride.compare(right.endpointOverride));
  result &= (0 == left.proxyHost.compare(right.proxyHost));
  result &= (0 == left.proxyUserName.compare(right.proxyUserName));
  result &= (0 == left.proxyPassword.compare(right.proxyPassword));
  result &= (0 == left.caPath.compare(right.caPath));
  result &= (0 == left.caFile.compare(right.caFile));

  result &= (left.requestTimeoutMs == right.requestTimeoutMs);
  result &= (left.connectTimeoutMs == right.connectTimeoutMs);
  result &= (left.maxConnections == right.maxConnections);
  result &= (left.proxyPort == right.proxyPort);
  result &= (left.useDualStack == right.useDualStack);
  result &= (left.enableClockSkewAdjustment == right.enableClockSkewAdjustment);
  result &= (left.followRedirects == right.followRedirects);
  result &= (left.verifySSL == right.verifySSL);

  return result;
}

bool operator!=(const ClientConfiguration & left, const ClientConfiguration & right)
{
  return !(left == right);
}

ClientConfigurationProvider::ClientConfigurationProvider(
  std::shared_ptr<ParameterReaderInterface> reader)
{
  this->reader_ = reader;
}

void ClientConfigurationProvider::PopulateUserAgent(Aws::String & user_agent,
                                                    std::string ros_version_override)
{
  Aws::String ros_user_agent_suffix = " exec-env/AWS_RoboMaker ros-" CMAKE_ROS_DISTRO "/";
  if (ros_version_override.empty()) {
    ros_user_agent_suffix += CMAKE_ROS_VERSION;
  } else {
    ros_user_agent_suffix += ros_version_override.c_str();
  }
  user_agent += ros_user_agent_suffix;
}

ClientConfiguration ClientConfigurationProvider::GetClientConfiguration(
  std::string ros_version_override)
{
  ClientConfiguration config;
  Aws::Config::AWSProfileProvider profile_provider;
  /**
   * Give priority to region parameter from the parameter reader if exists, otherwise use the AWS
   * SDK/CLI config file (defaults to ~/.aws/config). The latter is needed because unlike
   * credentials, region does not get loaded automatically from the profile by the SDK (it simply
   * defaults to us-east-1).
   */
  config.region = profile_provider.GetProfile().GetRegion();

  Aws::String temp_string;
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/region", temp_string)) {
    config.region = temp_string;
  } 
  PopulateUserAgent(config.userAgent, ros_version_override);
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/user_agent", temp_string)) {
    config.userAgent = temp_string;
  }
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/endpoint_override", temp_string)) {
    config.endpointOverride = temp_string;
  }
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/proxy_host", temp_string)) {
    config.proxyHost = temp_string;
  }
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/proxy_user_name", temp_string)) {
    config.proxyUserName = temp_string;
  }
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/proxy_password", temp_string)) {
    config.proxyPassword = temp_string;
  }
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/ca_path", temp_string)) {
    config.caPath = temp_string;
  }
  if (AWS_ERR_OK == reader_->ReadString(CLIENT_CONFIG_PREFIX "/ca_file", temp_string)) {
    config.caFile = temp_string;
  }

  int temp_int;
  if (AWS_ERR_OK == reader_->ReadInt(CLIENT_CONFIG_PREFIX "/request_timeout_ms", temp_int)) {
    config.requestTimeoutMs = temp_int;
  }
  if (AWS_ERR_OK == reader_->ReadInt(CLIENT_CONFIG_PREFIX "/connect_timeout_ms", temp_int)) {
    config.connectTimeoutMs = temp_int;
  }
  if (AWS_ERR_OK == reader_->ReadInt(CLIENT_CONFIG_PREFIX "/max_connections", temp_int)) {
    config.maxConnections = temp_int;
  }
  if (AWS_ERR_OK == reader_->ReadInt(CLIENT_CONFIG_PREFIX "/proxy_port", temp_int)) {
    config.proxyPort = temp_int;
  }

  bool temp_bool;
  if (AWS_ERR_OK == reader_->ReadBool(CLIENT_CONFIG_PREFIX "/use_dual_stack", temp_bool)) {
    config.useDualStack = temp_bool;
  }
  if (AWS_ERR_OK == reader_->ReadBool(CLIENT_CONFIG_PREFIX "/enable_clock_skew_adjustment", temp_bool)) {
    config.enableClockSkewAdjustment = temp_bool;
  }
  if (AWS_ERR_OK == reader_->ReadBool(CLIENT_CONFIG_PREFIX "/follow_redirects", temp_bool)) {
    config.followRedirects = temp_bool;
  }
  if (AWS_ERR_OK == reader_->ReadBool(CLIENT_CONFIG_PREFIX "/verify_SSL", temp_bool)) {
    config.verifySSL = temp_bool;
  }

  if (AWS_ERR_OK == reader_->ReadInt(CLIENT_CONFIG_PREFIX "/max_retries", temp_int)) {
    config.retryStrategy = std::make_shared<Aws::Client::DefaultRetryStrategy>(temp_int);
  }

  return config;
}

}  // namespace Client
}  // namespace Aws
