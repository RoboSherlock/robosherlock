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

#ifndef VIEW_NAMES_H_
#define VIEW_NAMES_H_

#define VIEW_CAMERA_INFO         "camera_info"
#define VIEW_CAMERA_INFO_HD      "camera_info_hd"


#define VIEW_COLOR_IMAGE         "color_image"
#define VIEW_COLOR_IMAGE_HD      "color_image_hd"
#define VIEW_DEPTH_IMAGE         "depth_image"
#define VIEW_DEPTH_IMAGE_HD      "depth_image_hd"
#define VIEW_OBJECT_IMAGE        "object_image"
#define VIEW_OBJECT_IMAGE_HD     "object_image_hd"

#define VIEW_DISPLAY_IMAGE       "display_image"

#define VIEW_FISHEYE_IMAGE        "fisheye_image"
#define VIEW_COLOR_CAMERA_INFO    "color_camera_info"
#define VIEW_FISHEYE_CAMERA_INFO  "fisheye_camera_info"

#define VIEW_CLOUD               "cloud"
#define VIEW_CLOUD_NON_NAN       "cloud_non_nan"
#define VIEW_CLOUD_DOWNSAMPLED   "cloud_downsampled"
#define VIEW_CLOUD_SUPERVOXEL    "cloud_supervoxel"
#define VIEW_NORMALS             "normals"

#define VIEW_MASK                "mask"
#define VIEW_MASK_HD             "mask_hd"

#define VIEW_THERMAL_CAMERA_INFO "camera_info_thermal"

#define VIEW_THERMAL_IMAGE       "thermal_image"
#define VIEW_THERMAL_FUSED       "thermal_fused"
#define VIEW_THERMAL_COLOR_IMAGE "thermal_color_image"
#define VIEW_THERMAL_DEPTH_IMAGE "thermal_depth_image"

#define VIEW_THERMAL_CLOUD       "thermal_cloud"
#define VIEW_THERMAL_NORMALS     "thermal_normals"

#define VIEW_OBJECT_MAP          "object_map"
#define VIEW_OBJECTS             "objects"
#define VIEW_SEMANTIC_MAP        "semantic_map"
#define VIEW_SCENE               "scene"

#endif /* VIEW_NAMES_H_ */
