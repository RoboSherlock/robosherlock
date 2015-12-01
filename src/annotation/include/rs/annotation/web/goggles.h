/*
 * Copyright (c) 2012, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IAI_RS_GOGGLES_GOGGLES_H_
#define IAI_RS_GOGGLES_GOGGLES_H_

#include <opencv2/opencv.hpp>
#include <boost/python.hpp>
#include <Python.h>
#include <ros/package.h>
#include<iostream>

namespace ggg
{
class GogglesResponse;
class Info;
}

namespace rs
{
namespace goggles
{

// why is this even a class?..because we are going to replace this with a python lib since cppnetlib is shit
class GogglesFetcher
{
public:

  boost::python::object pyClient;

  GogglesFetcher()
  {
    Py_Initialize();
    std::stringstream pkgPath;
    pkgPath << ros::package::getPath("robosherlock") << "/scripts";

    PyObject *sysPath = PySys_GetObject("path");
    PyList_Insert(sysPath, 0, PyString_FromString(pkgPath.str().c_str()));
    boost::python::object pyClModule =  boost::python::import("goggles_http_client");
    boost::python::dict pyClModuleDict = boost::python::extract<boost::python::dict>(pyClModule.attr("__dict__"));
    pyClient  = pyClModuleDict["GogglesClient"]();
  }

  ~GogglesFetcher()
  {
    Py_Finalize();
  }
   int upload_buffer_synch(const char *buffer, size_t size, std::string &response);
   int upload_image_synch(const cv::Mat image, std::string &response);
   int upload_image_synch(const cv::Mat image, std::string &response, std::vector<uchar> &buffer);
};

class GogglesParser
{
public:
  typedef std::pair <std::string, std::string> NamedLink;

  GogglesParser(std::string response);

  int get_number_replies();

  void split_replies();
  static std::vector<std::string> split_replies(ggg::GogglesResponse resp);
  static std::vector<std::string> split_replies(std::string message);

  std::string get_category(int idx) const;
  static std::string get_category(ggg::Info info);
  static std::string get_category(std::string reply);

  std::string get_title(int idx) const;
  static std::string get_title(ggg::Info info);
  static std::string get_title(std::string reply);

  std::vector<int> get_bbox(int idx) const;
  static std::vector<int> get_bbox(ggg::Info info);
  static std::vector<int> get_bbox(std::string reply);

  std::vector<NamedLink> collect_links(int idx) const;
  static std::vector<NamedLink> collect_links(ggg::Info info);
  static std::vector<NamedLink> collect_links(std::string reply);

  std::string get_preview_link(int idx) const;
  static std::string get_preview_link(ggg::Info info);
  static std::string get_preview_link(std::string reply);

private:
  std::string response_;
  std::vector<std::string> response_fields_;
};

} // namespace goggles
} // namespace iai_rs
#endif // IAI_RS_GOGGLES_GOGGLES_H_

