/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <robosherlock/conversion/conversion.h>
#include <robosherlock/types/annotation_types.h>
#include <robosherlock/segmentation/ImageSegmentation.h>

namespace rs
{
namespace conversion
{

template<>
void from(const uima::FeatureStructure &fs, ImageSegmentation::Segment &output)
{
  rs::Segment seg(fs);
  output.contour.resize(seg.contour.size());
  for(size_t i = 0; i < output.contour.size(); ++i)
  {
    from(seg.contour.get(i), output.contour[i]);
  }

  if(seg.children.has())
  {
    output.children.resize(seg.children.size());
    for(size_t i = 0; i < output.children.size(); ++i)
    {
      from(seg.children.get(i), output.children[i]);
    }
  }
  else
  {
    output.children.resize(0);
  }

  output.area = seg.area();
  output.childrenArea = seg.childrenArea();
  output.holes = seg.holes();

  from(seg.rect(), output.rect);
  from(seg.moments(), output.moments);
  output.huMoments = seg.huMoments();

  from(seg.center(), output.center);
  from(seg.axisX(), output.axisX);
  from(seg.axisY(), output.axisY);
  output.alpha = seg.alpha();

  from(seg.rotation(), output.rotation);
  from(seg.translation(), output.translation);
  output.lengthX = seg.lengthX();
  output.lengthY = seg.lengthY();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const ImageSegmentation::Segment &input)
{
  rs::Segment seg = rs::create<rs::Segment>(cas);
  seg.contour.allocate(input.contour.size());
  for(size_t i = 0; i < input.contour.size(); ++i)
  {
    seg.contour.set(i, to(cas, input.contour[i]));
  }

  seg.children.allocate(input.children.size());
  for(size_t i = 0; i < input.children.size(); ++i)
  {
    seg.children.set(i, to(cas, input.children[i]));
  }

  seg.area(input.area);
  seg.childrenArea(input.childrenArea);
  seg.holes(input.holes);

  seg.rect(to(cas, input.rect));
  seg.moments(to(cas, input.moments));
  seg.huMoments(input.huMoments);

  seg.center(to(cas, input.center));
  seg.axisX(to(cas, input.axisX));
  seg.axisY(to(cas, input.axisY));
  seg.alpha(input.alpha);

  seg.rotation(to(cas, input.rotation));
  seg.translation(to(cas, input.translation));
  seg.lengthX(input.lengthX);
  seg.lengthY(input.lengthY);
  return seg;
}

}
}
