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

#include <aws_common/sdk_utils/parameter_reader.h>
#include <gmock/gmock.h>

namespace Aws {
namespace Client {

class ParameterReaderMock : public ParameterReaderInterface 
{
public:
  MOCK_CONST_METHOD2(ReadList, Aws::AwsError(const char *, std::vector<std::string> &));
  MOCK_CONST_METHOD2(ReadDouble, Aws::AwsError(const char *, double &));
  MOCK_CONST_METHOD2(ReadInt, Aws::AwsError(const char *, int &));
  MOCK_CONST_METHOD2(ReadBool, Aws::AwsError(const char *, bool &));
  MOCK_CONST_METHOD2(ReadString, Aws::AwsError(const char *, Aws::String &));
  MOCK_CONST_METHOD2(ReadStdString, Aws::AwsError(const char *, std::string &));
  MOCK_CONST_METHOD2(ReadMap, Aws::AwsError(const char *, std::map<std::string, std::string> &));
};

}  // namespace Client
}  // namespace Aws
