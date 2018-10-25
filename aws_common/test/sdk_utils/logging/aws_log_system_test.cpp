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

#include <aws_common/sdk_utils/logging/aws_log_system.h>
#include <gtest/gtest.h>

#include <iostream>

using namespace Aws::Utils::Logging;

class ClassUnderTest : public AWSLogSystem
{
private:
  int log_info_count = 0;
  int log_debug_count = 0;
  int log_trace_count = 0;
  int log_warn_count = 0;
  int log_error_count = 0;
  int log_fatal_count = 0;

  std::string expected_log_message_string;
  std::string expected_tag;

public:
  explicit ClassUnderTest(Aws::Utils::Logging::LogLevel log_level) : AWSLogSystem(log_level) {}

  ~ClassUnderTest() = default;

  /*********************************
   * Overidden log methods to test *
   *********************************/

  void LogInfo(const char * tag, const std::string & message) override
  {
    log_info_count++;
    expected_log_message_string = message;
    expected_tag = tag;
  }

  void LogTrace(const char * tag, const std::string & message) override
  {
    log_trace_count++;
    expected_log_message_string = message;
    expected_tag = tag;
  }

  void LogDebug(const char * tag, const std::string & message) override
  {
    log_debug_count++;
    expected_log_message_string = message;
    expected_tag = tag;
  }

  void LogWarn(const char * tag, const std::string & message) override
  {
    log_warn_count++;
    expected_log_message_string = message;
    expected_tag = tag;
  }

  void LogError(const char * tag, const std::string & message) override
  {
    log_error_count++;
    expected_log_message_string = message;
    expected_tag = tag;
  }

  void LogFatal(const char * tag, const std::string & message) override
  {
    log_fatal_count++;
    expected_log_message_string = message;
    expected_tag = tag;
  }

  /*********************************
   *     static helper methods     *
   *********************************/

  static const std::string & GetExpectedLogMessageString(const ClassUnderTest * class_under_test)
  {
    return class_under_test->expected_log_message_string;
  }

  static const std::string & GetExpectedTag(const ClassUnderTest * class_under_test)
  {
    return class_under_test->expected_tag;
  }

  static int GetLogInfoCount(const ClassUnderTest * class_under_test)
  {
    return class_under_test->log_info_count;
  }

  static int GetLogDebugCount(const ClassUnderTest * class_under_test)
  {
    return class_under_test->log_debug_count;
  }

  static int GetLogWarnCount(const ClassUnderTest * class_under_test)
  {
    return class_under_test->log_warn_count;
  }

  static int GetLogErrorCount(const ClassUnderTest * class_under_test)
  {
    return class_under_test->log_error_count;
  }

  static int GetLogFatalCount(const ClassUnderTest * class_under_test)
  {
    return class_under_test->log_fatal_count;
  }

  static void SetExpectedLogMessageString(ClassUnderTest * class_under_test,
                                          const std::string & message)
  {
    class_under_test->expected_log_message_string = message;
  }

  static void SetExpectedTagString(ClassUnderTest * class_under_test, const std::string & tag)
  {
    class_under_test->expected_tag = tag;
  }

  static void ResetLogCounts(ClassUnderTest * class_under_test)
  {
    class_under_test->log_info_count = 0;
    class_under_test->log_debug_count = 0;
    class_under_test->log_trace_count = 0;
    class_under_test->log_warn_count = 0;
    class_under_test->log_error_count = 0;
    class_under_test->log_fatal_count = 0;
  }
};

TEST(TestAWSLogSystem, TestLogMethod)
{
  // Create a logger object with "Debug" configured level.
  // This means, log at all the levels above "Debug" will be emitted.
  ClassUnderTest * logger = new ClassUnderTest(Aws::Utils::Logging::LogLevel::Debug);

  // Test if info log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Info, "info_tag", "[%d] fake info log message", 1);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "info_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[1] fake info log message");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 1);

  // Test if debug log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Debug, "debug_tag", "[%d] fake debug log message", 2);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "debug_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[2] fake debug log message");
  EXPECT_EQ(ClassUnderTest::GetLogDebugCount(logger), 1);

  // Test if warn log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Warn, "warn_tag", "[%d] fake warn log message", 3);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "warn_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[3] fake warn log message");
  EXPECT_EQ(ClassUnderTest::GetLogWarnCount(logger), 1);

  // Test if error log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Error, "error_tag", "[%d] fake error log message", 4);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "error_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[4] fake error log message");
  EXPECT_EQ(ClassUnderTest::GetLogErrorCount(logger), 1);

  // Test if fatal log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Fatal, "fatal_tag", "[%d] fake fatal log message", 5);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "fatal_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[5] fake fatal log message");
  EXPECT_EQ(ClassUnderTest::GetLogFatalCount(logger), 1);

  // Test if debug log function is correctly getting called second time with appropriate log
  // message.
  logger->Log(Aws::Utils::Logging::LogLevel::Debug, "debug_tag", "[%d] fake debug log message", 6);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "debug_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[6] fake debug log message");
  EXPECT_EQ(ClassUnderTest::GetLogDebugCount(logger), 2);

  delete logger;

  // Create a logger object with "Error" configured level.
  // This means, logs only above "Error" level will be emitted.
  logger = new ClassUnderTest(Aws::Utils::Logging::LogLevel::Error);

  // info log function should not be called since info_log_level < error_log_level
  logger->Log(Aws::Utils::Logging::LogLevel::Info, "info_tag", "[%d] fake info log message", 1);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(), "");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 0);

  // debug log function should not be called since debug_log_level < error_log_level
  logger->Log(Aws::Utils::Logging::LogLevel::Debug, "debug_tag", "[%d] fake debug log message", 2);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(), "");
  EXPECT_EQ(ClassUnderTest::GetLogDebugCount(logger), 0);

  // Test if error log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Error, "error_tag", "[%d] fake error log message", 3);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "error_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[3] fake error log message");
  EXPECT_EQ(ClassUnderTest::GetLogErrorCount(logger), 1);

  // Test if fatal log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Fatal, "fatal_tag", "[%d] fake fatal log message", 4);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "fatal_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[4] fake fatal log message");
  EXPECT_EQ(ClassUnderTest::GetLogFatalCount(logger), 1);

  delete logger;
}

TEST(TestAWSLogSystem, TestLogStreamMethod)
{
  Aws::OStringStream messageStream;

  // Create a logger object with "Debug" configured level.
  // This means, log at all the levels above "Debug" will be emitted.
  ClassUnderTest * logger = new ClassUnderTest(Aws::Utils::Logging::LogLevel::Debug);

  // Test if info log function is correctly getting called with correct log message.
  messageStream << "This is a first info log message.";
  logger->LogStream(Aws::Utils::Logging::LogLevel::Info, "info_tag", messageStream);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "info_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "This is a first info log message.");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 1);

  messageStream << " This is a second info log message.";
  logger->LogStream(Aws::Utils::Logging::LogLevel::Info, "info_tag", messageStream);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "info_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "This is a first info log message. This is a second info log message.");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 2);

  delete logger;
  messageStream.str("");

  // Create a logger object with "Error" configured level.
  // This means, logs only above "Error" level will be emitted.
  logger = new ClassUnderTest(Aws::Utils::Logging::LogLevel::Error);

  // info log function should not be called since info_log_level < error_log_level
  messageStream << "This is a first info log message.";
  logger->LogStream(Aws::Utils::Logging::LogLevel::Info, "info_tag", messageStream);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(), "");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 0);

  // debug log function should not be called since debug_log_level < error_log_level
  logger->LogStream(Aws::Utils::Logging::LogLevel::Debug, "debug_tag", messageStream);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(), "");
  EXPECT_EQ(ClassUnderTest::GetLogDebugCount(logger), 0);

  // clear string buffer and expected log message string.
  messageStream.str("");
  ClassUnderTest::SetExpectedLogMessageString(logger, "");

  // Test if error log function is correctly getting called with correct log message.
  messageStream << "This is a first error log message.";
  logger->LogStream(Aws::Utils::Logging::LogLevel::Error, "error_tag", messageStream);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "error_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "This is a first error log message.");
  EXPECT_EQ(ClassUnderTest::GetLogErrorCount(logger), 1);

  delete logger;
}

TEST(TestAWSLogSystem, TestChangingLogLevels)
{
  // Create a logger object with "Debug" configured level.
  // This means, log at all the levels above "Debug" will be emitted.
  ClassUnderTest * logger = new ClassUnderTest(Aws::Utils::Logging::LogLevel::Debug);

  // check if log level is correctly set.
  EXPECT_EQ(logger->GetLogLevel(), Aws::Utils::Logging::LogLevel::Debug);

  // Test if info log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Info, "info_tag", "[%d] fake info log message", 1);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "info_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[1] fake info log message");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 1);

  // reset test variables
  ClassUnderTest::SetExpectedLogMessageString(logger, "");
  ClassUnderTest::SetExpectedTagString(logger, "");
  ClassUnderTest::ResetLogCounts(logger);

  // change log level at runtime
  logger->SetLogLevel(Aws::Utils::Logging::LogLevel::Error);

  // check if log level is correctly set.
  EXPECT_EQ(logger->GetLogLevel(), Aws::Utils::Logging::LogLevel::Error);

  // info log function should not be called since info_log_level < error_log_level
  logger->Log(Aws::Utils::Logging::LogLevel::Info, "info_tag", "[%d] fake info log message", 1);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(), "");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 0);

  // Test if fatal log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Fatal, "fatal_tag", "[%d] fake fatal log message", 1);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "fatal_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[1] fake fatal log message");
  EXPECT_EQ(ClassUnderTest::GetLogFatalCount(logger), 1);

  // reset test variables
  ClassUnderTest::SetExpectedLogMessageString(logger, "");
  ClassUnderTest::SetExpectedTagString(logger, "");
  ClassUnderTest::ResetLogCounts(logger);

  // now change log level to Info.
  logger->SetLogLevel(Aws::Utils::Logging::LogLevel::Info);

  // check if log level is correctly set.
  EXPECT_EQ(logger->GetLogLevel(), Aws::Utils::Logging::LogLevel::Info);

  // Test if info log function is correctly getting called with correct log message.
  logger->Log(Aws::Utils::Logging::LogLevel::Info, "info_tag", "[%d] fake info log message", 3);
  EXPECT_STREQ(ClassUnderTest::GetExpectedTag(logger).c_str(), "info_tag");
  EXPECT_STREQ(ClassUnderTest::GetExpectedLogMessageString(logger).c_str(),
               "[3] fake info log message");
  EXPECT_EQ(ClassUnderTest::GetLogInfoCount(logger), 1);

  delete logger;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
