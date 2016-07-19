#pragma once
#ifndef __exception_h__
#define __exception_h__

#include <exception>
#include <string>
#include <stdlib.h>

// ROS
#include <ros/ros.h>

namespace rs
{

class Exception : public std::exception
{
protected:
  const std::string message;

public:
  Exception(const std::string &message) throw() : std::exception(), message(message)
  {
  }

  virtual ~Exception() throw()
  {
  }

  virtual const char *what() const throw()
  {
    return message.c_str();
  }
};

class FrameFilterException : public Exception
{
protected:
  const std::string message;

public:
  FrameFilterException() throw() : Exception("This frame has been filtered, no further processing!")
  {
  }

  virtual ~FrameFilterException() throw()
  {
  }
};

}

#define EXCEPTION_FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define EXCEPTION_STR_HELPER(str) #str
#define EXCEPTION_STR(str) EXCEPTION_STR_HELPER(str)

#define throw_exception_message(MESSAGE) throw rs::Exception(std::string("") + EXCEPTION_FILENAME + "(" + EXCEPTION_STR(__LINE__) + ")[" + __func__ + "] " + MESSAGE);

#define check_expression(EXPRESSION, MESSAGE) if(!(EXPRESSION)) throw_exception_message(MESSAGE)

#define check_ros() check_expression(ros::ok(), "ROS was shutdown while processing.")

#endif //__exception_h__
