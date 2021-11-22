/*
 * Copyright (c) 2012, Nico Blodow <blodow@cs.tum.edu>
 * Refactoring by Ferenc Balint-Benczedi 2015
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

#include <uima/api.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/annotation/web/goggles.h>

#include <robosherlock/DrawingAnnotator.h>

using namespace uima;

class GogglesAnnotator : public DrawingAnnotator
{

private:
  rs::goggles::GogglesFetcher gf;

  typedef std::map <int, std::string> CacheMap;
  typedef std::map <int, int> CacheHitMap;
  CacheMap goggles_cache;
  CacheMap goggles_hash_cache;
  CacheHitMap goggles_cache_hits;

  cv::Mat disp;

public:
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  GogglesAnnotator(): DrawingAnnotator(__func__)
  {

  }

  std::string questionable_string_fixing(std::string input)
  {
    std::string output = input;
    std::string::iterator it;
    for(it = output.begin(); it != output.end(); ++it)
      if(((unsigned char)*it) > 127)
      {
        *it = '?';
      }

    return output;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process begins");
    rs::StopWatch clock;
    int positive_hits = 0;
    int negative_hits = 0;
    std::vector<double> timings;

    // create scene cas wrapper for cas and get kinect frame
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::ObjectHypothesis> clusters;
    cv::Mat color;
    cas.get(VIEW_COLOR_IMAGE_HD, color);
    disp = color.clone();
    scene.identifiables.filter(clusters);
    for(std::vector<rs::ObjectHypothesis>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
      rs::ImageROI image_rois = it->rois.get();

      std::string response;

      //TODO: add check for previous cluster
      rs::StopWatch goggles_clock;
      outInfo("looking it up in Goggles..");

      cv::Mat image, mask;
      cv::Rect roi;
      rs::conversion::from(image_rois.roi_hires(), roi);
      rs::conversion::from(image_rois.mask_hires(), mask);

      color(roi).copyTo(image, mask);

      std::vector<uchar> jpeg_buffer;
      int http_response = gf.upload_image_synch(image, response, jpeg_buffer);

      outInfo("HTTP response: " << http_response);
      timings.push_back(goggles_clock.getTime());


      rs::goggles::GogglesParser gp(response);

      outInfo("Result contains " << gp.get_number_replies() << " fields:");

      for(unsigned int i = 0; i < gp.get_number_replies(); ++i)
      {
        rs::Goggles annotation = rs::create<rs::Goggles>(tcas);
        annotation.category.set(questionable_string_fixing(gp.get_category(i)));
        outInfo("Category:" << gp.get_category(i) << "  Title: " << gp.get_title(i));
        annotation.title.set(questionable_string_fixing(gp.get_title(i)));
        drawRestuls(roi, gp.get_category(i),gp.get_title(i),i);
        std::vector<int> bbox = gp.get_bbox(i);
        if(bbox.size() == 4)
        {
          std::vector<float> bbox_f(bbox.size());
          for(unsigned int j = 0; j < bbox.size(); ++j)
          {
            bbox_f[j] = bbox[j];
          }
          annotation.bbox.set(bbox_f);
        }

        std::string preview_link = gp.get_preview_link(i);
        if(preview_link.size() > 0)
        {
          annotation.preview_link.set(preview_link);
        }

        outInfo("Preview Link: " << preview_link);

        std::vector<rs::goggles::GogglesParser::NamedLink> links;
        links = gp.collect_links(i);

        std::vector<rs::NamedLink> uima_links;
        for(unsigned int j = 0; j < links.size(); ++j)
        {
          std::string link_name = questionable_string_fixing(links[j].first);
          std::string link_url = links[j].second;

          rs::NamedLink nl = rs::create<rs::NamedLink>(tcas);
          nl.name.set(link_name);
          nl.url.set(link_url);
          uima_links.push_back(nl);

          outInfo("Link (\"" << link_name << "\"): " << link_url);
        }

        annotation.links.set(uima_links);

        it->annotations.append(annotation);
      }

      if(gp.get_number_replies() > 0)
      {
        outInfo("¸.·´¯`·.´¯`·.¸¸.·´¯`·.¸><(((º>");
        outInfo("found goggles response for ID "); //<< " :" << response );
        outInfo("<º)))><¸.·´¯`·.´¯`·.¸¸.·´¯`·.¸");

        positive_hits++;
      }
      else
      {
        negative_hits++;
      }
    }

    return UIMA_ERR_NONE;
  }

  void drawRestuls(cv::Rect roi,std::string cat, std::string title, int index)
  {
      cv::rectangle(disp,roi,cv::Scalar(200,0,0),2);
      int baseLine;
      cv::Size textSize = cv::getTextSize(cat+":"+title,cv::FONT_HERSHEY_PLAIN,1.5,2.0,&baseLine);
      cv::putText(disp,cat+":"+title,cv::Point(roi.x,roi.y-textSize.height - index*17),cv::FONT_HERSHEY_PLAIN,1.5,2.0);
  }

  void drawImageWithLock(cv::Mat &d)
  {
    d = disp.clone();
  }
  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(GogglesAnnotator)
