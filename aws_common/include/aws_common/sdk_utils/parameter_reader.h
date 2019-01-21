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
#pragma once
#include <aws/core/client/ClientConfiguration.h>
#include <aws/core/utils/StringUtils.h>
#include <aws_common/sdk_utils/aws_error.h>

namespace Aws {
namespace Client {

class ParameterPath {
public:
    /**
     * Legacy constructor for backwards compatibility.
     * @hint For ROS1, the node namespace separator is the same as the parameter namespace separator,
     *     and so the path would simply be the given string (dictated by Ros1NodeParameterReader).
     *   For ROS2 the separators differ; If only node namespace separators were found, we would treat them as parameter separators.
     * @param name Parameter name, which in practice would be the path of the parameter, e.g. config/timeoutMs.
     */
    ParameterPath(const char * name) : name_(name) {}

    /**
     * @example The parameter under node "lidar_node" in namespace "sensor_nodes",
     * with parameter namespace "settings" and parameter key "frequency", should be specified as follows:
     *   node_namespace: ["sensor_nodes", "lidar_node"]
     *   parameter_path_keys: ["settings", "frequency"]
     * @note Node namespaces should be empty if used by a node looking for a parameter of its own (a local parameter).
     * @param node_namespaces
     * @param parameter_path_keys
     */
    ParameterPath(const std::vector<std::string> & node_namespaces, const std::vector<std::string> & parameter_path_keys) :
      node_namespaces_(node_namespaces), parameter_path_keys_(parameter_path_keys) {}

    /**
     * @example The local parameter "timeout" in parameter namespace "config" should be specified as follows:
     *   ParameterPath("config", "timeout")
     * @tparam Args
     * @param parameter_path_keys
     */
    template<typename ...Args>
    ParameterPath(Args... parameter_path_keys) :
      parameter_path_keys_(std::vector<std::string>{(parameter_path_keys)...}) {}

    /**
     * @note Only applies if node_namespaces was specified during construction; otherwise, an empty string is returned.
     * @param node_namespace_separator
     * @return string The parameter's whereabouts in the node namespace hierarchy.
     */
    std::string get_node_path(char node_namespace_separator) const
    {
        std::string resolved_path;
        /* Construct the node's path by the provided lists of keys */
        for (auto key = node_namespaces_.begin(); key != node_namespaces_.end(); key++) {
            resolved_path += *key + node_namespace_separator;
        }
        if (!resolved_path.empty() && resolved_path.back() == node_namespace_separator) {
            resolved_path.pop_back();
        }
        return resolved_path;
    }
    /**
     * @note Only applies if parameter_path_keys was specified during construction; otherwise, an empty string is returned.
     * @param parameter_namespace_separator
     * @return string The parameter path including parameter namespaces but excluding node's namespaces.
     */
    std::string get_local_path(char parameter_namespace_separator) const
    {
        std::string resolved_path;
        /* Construct the parameter's path by the provided lists of keys */
        for (auto key = parameter_path_keys_.begin(); key != parameter_path_keys_.end(); key++) {
            resolved_path += *key + parameter_namespace_separator;
        }
        if (!resolved_path.empty() && resolved_path.back() == parameter_namespace_separator) {
            resolved_path.pop_back();
        }
        return resolved_path;
    }
    /**
     * Resolves & returns the parameter's path. Supports two separate use cases:
     *  1. A single, 'flat' string was provided upon construction.
     *    In this case, we return it as is.
     *  2. Detailed lists of strings were specified for the parameter & node namespaces.
     *    In this case, we construct the resolved path using the provided separators.
     * @param node_namespace_separator
     * @param parameter_namespace_separator
     * @return string representing the full, resolved path of the parameter.
     * @note If node_namespaces and parameter_path_keys are empty, an empty string would be returned.
     */
    std::string get_resolved_path(char node_namespace_separator,
                                  char parameter_namespace_separator) const
    {
      if (!name_.empty()) {
        return name_;
      } else {
          std::string resolved_path = get_node_path(node_namespace_separator);
          if (!resolved_path.empty()) {
              resolved_path += node_namespace_separator;
          }
          resolved_path += get_local_path(parameter_namespace_separator);
          return resolved_path;
      }
    }
private:
    /**
     * Member variables to store the parameter's path in the namespace hierarchy.
     * parameter_path_keys_ Parameter namespaces list.
     * node_namespaces_ Node namespaces list.
     * name_ A single string. Used when simpler initialization is required, instead of the aforementioned namespace lists.
     */
    const std::vector<std::string> parameter_path_keys_;
    const std::vector<std::string> node_namespaces_;
    mutable std::string name_;
};

class ParameterReaderInterface
{
public:
  virtual ~ParameterReaderInterface() = default;

  /**
   * read a list from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'double' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadList(const ParameterPath & parameter_path, std::vector<std::string> & out) const = 0;

  /**
   * read a double value from the provided parameter name
   * @param parameter_path an object representing the path of the parameter to be read
   * @param out the output of 'double' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadDouble(const ParameterPath & parameter_path, double & out) const = 0;

  /**
   * read an integer value from the provided parameter name
   * @param parameter_path an object representing the path of the parameter to be read
   * @param out the output of 'int' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadInt(const ParameterPath & parameter_path, int & out) const = 0;

  /**
   * read a boolean value from the provided parameter name
   * @param parameter_path an object representing the path of the parameter to be read
   * @param out the output of 'bool' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadBool(const ParameterPath & parameter_path, bool & out) const = 0;

  /**
   * read a string value from the provided parameter name
   * @param parameter_path an object representing the path of the parameter to be read
   * @param out the output of 'Aws::String' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadString(const ParameterPath & parameter_path, Aws::String & out) const = 0;

  /**
   * read a string from the provided parameter name
   * @param parameter_path an object representing the path of the parameter to be read
   * @param out the output of 'std::string' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadStdString(const ParameterPath & parameter_path, std::string & out) const = 0;

  /**
   * read a map from the provided parameter name
   * @param parameter_path an object representing the path of the parameter to be read
   * @param out the output of 'std::map' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadMap(const ParameterPath & parameter_path, std::map<std::string, std::string> & out) const = 0;
};

}  // namespace Client
}  // namespace Aws
