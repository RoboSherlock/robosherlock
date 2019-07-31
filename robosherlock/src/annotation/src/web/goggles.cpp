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

#include <stdlib.h>
#include <rs/annotation/web/goggles.h>
#include <request.pb.h>
#include <response.pb.h>



#include <fstream>

namespace rs
{
namespace goggles
{

int GogglesFetcher::upload_image_synch(const cv::Mat image, std::string &response)
{
  std::vector<uchar> buffer;
  return upload_image_synch(image, response, buffer);
}

int GogglesFetcher::upload_image_synch(const cv::Mat image, std::string &response, std::vector<uchar> &buffer)
{
  if(cv::imencode(std::string(".jpg"), image, buffer/*, const vector<int>& params=vector<int>()*/))
  {
    return upload_buffer_synch((char *)&buffer[0], buffer.size(), response);
  }
  else
  {
    return -1;
  }
}

//calling the service is handled through a python script since cppnetlib kept crashing
int GogglesFetcher::upload_buffer_synch(const char *buffer, size_t size, std::string &response)
{

  int status = -1;

  // put image bytes into protobuffer object
  ggg::GogglesRequest request;
  request.mutable_wrap_img()->mutable_image()->set_image_bytes((const char *) buffer, size);
  request.mutable_lang()->set_string1("en");
  request.mutable_lang()->set_string2("US");
  request.mutable_lang()->mutable_enable_ui()->set_val1(1);
  request.mutable_lang()->mutable_enable_ui()->set_val2(1);

  // get serialized buffer
  std::string str;
  request.SerializeToString(&str);
  boost::python::object resp = pyClient.attr("call_service")( boost::python::str(str.c_str(),str.size()));

  //TODO: fix this to get a status of the message. e.g 200 if OK
  boost::python::extract<std::string> respStr (resp);
  response = std::string(respStr);
  return status;
}

/**
 * GogglesParser contains methods to parse goggles responses from protobuffer
 * strings and from message objects like ggg::GogglesResponse. All functions
 * like get_category etc. are available as static functions which need no
 * knowledge of the ggg::* classes, as well as stateful member functions which
 * are slightly more optimal.
 */
GogglesParser::GogglesParser(std::string response)
  : response_(response)
{
  split_replies();
}

int GogglesParser::get_number_replies()
{
  return response_fields_.size();
}

/**
 * split_replies takes a goggles response buffer and splits it into a vector
 * of strings, either stored in response_fields_ or returned
 */
void GogglesParser::split_replies()
{
  response_fields_ = split_replies(response_);
}
std::vector<std::string> GogglesParser::split_replies(ggg::GogglesResponse resp)
{
  std::vector<std::string> res;
  for(unsigned int i = 0; i < resp.info_size(); i++)
  {
    std::string info_str;
    resp.info(i).SerializeToString(&info_str);
    res.push_back(info_str);
  }

  return res;
}
std::vector<std::string> GogglesParser::split_replies(std::string message)
{
  ggg::GogglesResponse r;
  r.ParseFromString(message);
  return split_replies(r);
}

/**
 * get_category
 */
std::string GogglesParser::get_category(int idx) const
{
  return get_category(response_fields_[idx]);
}
std::string GogglesParser::get_category(ggg::Info info)
{
  if(!info.has_uistuff() || !info.uistuff().has_result_description())
  {
    return std::string();
  }
  return std::string(info.uistuff().result_description());
}
std::string GogglesParser::get_category(std::string reply)
{
  ggg::Info info;
  info.ParseFromString(reply);
  return get_category(info);
}

/**
 * get_title
 */
std::string GogglesParser::get_title(int idx) const
{
  return get_title(response_fields_[idx]);
}
std::string GogglesParser::get_title(ggg::Info info)
{
  if(!info.has_uistuff() || !info.uistuff().has_result_string())
  {
    return std::string();
  }
  return std::string(info.uistuff().result_string());
}
std::string GogglesParser::get_title(std::string reply)
{
  ggg::Info info;
  info.ParseFromString(reply);
  return get_title(info);
}

/**
 * get_bbox
 */
std::vector<int> GogglesParser::get_bbox(int idx) const
{
  return get_bbox(response_fields_[idx]);
}
std::vector<int> GogglesParser::get_bbox(ggg::Info info)
{
  if(!info.has_uistuff() || !info.uistuff().has_coords())
  {
    return std::vector<int> ();
  }

  std::vector<int> res(4);
  ggg::Coords c = info.uistuff().coords();
  res[0] = c.x();
  res[1] = c.y();
  res[2] = c.width();
  res[3] = c.height();
  return res;
}
std::vector<int> GogglesParser::get_bbox(std::string reply)
{
  ggg::Info info;
  info.ParseFromString(reply);
  return get_bbox(info);
}

/**
 * collect_links
 */
std::vector<GogglesParser::NamedLink> GogglesParser::collect_links(int idx) const
{
  return collect_links(response_fields_[idx]);
}
std::vector<GogglesParser::NamedLink> GogglesParser::collect_links(ggg::Info info)
{
  if(!info.has_uistuff())
  {
    return std::vector<GogglesParser::NamedLink> ();
  }

  ggg::UIStuff uistuff = info.uistuff();

  std::vector<GogglesParser::NamedLink> res;

  // get all actions from this Info block
  if(uistuff.has_actions())
  {
    ggg::ActionList al = uistuff.actions();
    for(int i = 0; i < al.action_size(); ++i)
    {
      ggg::Action a = al.action(i);
      if(a.has_action_title() && a.has_url())
      {
        res.push_back(NamedLink(a.action_title(), a.url()));
      }
    }
  }

  // get search_more field
  if(uistuff.has_search_more())
  {
    res.push_back(NamedLink("Search More", uistuff.search_more()));
  }

  // get image urls
  if(uistuff.has_image_urls())
  {
    ggg::ImageURLs urls = uistuff.image_urls();
    if(urls.has_image_highres())
    {
      res.push_back(NamedLink("High-Res Image", urls.image_highres()));
    }
    if(urls.has_image_site())
    {
      res.push_back(NamedLink("Found on Page", urls.image_site()));
    }
  }

  return res;
}
std::vector<GogglesParser::NamedLink> GogglesParser::collect_links(std::string reply)
{
  ggg::Info info;
  info.ParseFromString(reply);
  return collect_links(info);
}

/**
 * get_preview_link
 */
std::string GogglesParser::get_preview_link(int idx) const
{
  return get_preview_link(response_fields_[idx]);
}
std::string GogglesParser::get_preview_link(ggg::Info info)
{
  if(!info.has_uistuff() || !info.uistuff().has_image_urls() || !info.uistuff().image_urls().has_image_show())
  {
    return std::string();
  }
  return std::string(info.uistuff().image_urls().image_show());
}
std::string GogglesParser::get_preview_link(std::string reply)
{
  ggg::Info info;
  info.ParseFromString(reply);
  return get_preview_link(info);
}

} // namespace goggles
} // namespace iai_rs

