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
  virtual AwsError ReadList(const char * name, std::vector<std::string> & out) const = 0;

  /**
   * read a double value from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'double' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadDouble(const char * name, double & out) const = 0;

  /**
   * read an integer value from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'int' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadInt(const char * name, int & out) const = 0;

  /**
   * read a boolean value from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'bool' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadBool(const char * name, bool & out) const = 0;

  /**
   * read a string value from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'Aws::String' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadString(const char * name, Aws::String & out) const = 0;

  /**
   * read a string from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'std::string' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadStdString(const char * name, std::string & out) const = 0;

  /**
   * read a map from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'std::map' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   */
  virtual AwsError ReadMap(const char * name, std::map<std::string, std::string> & out) const = 0;
};

}  // namespace Client
}  // namespace Aws
