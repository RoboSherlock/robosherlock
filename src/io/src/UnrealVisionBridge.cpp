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

// STD
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <immintrin.h>

// RS
#include <rs/io/UnrealVisionBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

UnrealVisionBridge::UnrealVisionBridge(const boost::property_tree::ptree &pt) : CamInterface(pt), sizeRGB(3 * sizeof(uint8_t)), sizeFloat(sizeof(uint16_t)), running(false), isConnected(false)
{
#ifdef __F16C__
  readConfig(pt);

  const size_t bufferSize = 1024 * 1024 * 10;
  bufferComplete.resize(bufferSize);
  bufferActive.resize(bufferSize);
  bufferInUse.resize(bufferSize);

  outInfo("starting receiver and transmitter threads.");
  running = true;
  receiver = std::thread(&UnrealVisionBridge::receive, this);
#else
  outError("F16C not supported. Use of UnrealBridge is not possible");
  exit(1);
#endif
}

UnrealVisionBridge::~UnrealVisionBridge()
{
}

void UnrealVisionBridge::readConfig(const boost::property_tree::ptree &pt)
{
  address = pt.get<std::string>("server.address", "127.0.0.1");
  port = (uint16_t)pt.get<int>("server.port", 10000);
  tfFrom = pt.get<std::string>("tf.from", "unreal_vision_optical_frame");
  tfTo = pt.get<std::string>("tf.to", "map");

  outInfo("Address: " FG_BLUE << address);
  outInfo("   Port: " FG_BLUE << port);
  outInfo("TF From: " FG_BLUE << tfFrom);
  outInfo("  TF To: " FG_BLUE << tfTo);
}

void UnrealVisionBridge::convertDepth(const uint16_t *in, __m128 *out) const
{
#ifdef __F16C__
  const size_t size = (packet.header.width * packet.header.height) / 4;
  for(size_t i = 0; i < size; ++i, in += 4, ++out)
  {
    *out = _mm_cvtph_ps(_mm_set_epi16(0, 0, 0, 0, *(in + 3), *(in + 2), *(in + 1), *(in + 0)));
  }
#endif
}

void UnrealVisionBridge::connectToServer()
{
  outInfo("creating socket.");
  connection = socket(AF_INET, SOCK_STREAM, 0);
  if(connection < 0)
  {
    outError("could not open socket");
    return;
  }

  struct sockaddr_in serverAddress;
  bzero((char *) &serverAddress, sizeof(serverAddress));
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = inet_addr(address.c_str());
  serverAddress.sin_port = htons(port);

  outInfo("connecting to server.");
  while(running && ros::ok())
  {
    if(connect(connection, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) >= 0)
    {
      isConnected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if(!running || !ros::ok())
  {
    return;
  }

  int receiveBufferSize = 1024 * 1024 * 10;
  socklen_t optionLength = sizeof(int);
  if(setsockopt(connection, SOL_SOCKET, SO_RCVBUF, (void *)&receiveBufferSize, optionLength) < 0)
  {
    //outWarn("could not set socket receive buffer size to: " << receiveBufferSize);
  }

  if(getsockopt(connection, SOL_SOCKET, SO_RCVBUF, (void *)&receiveBufferSize, &optionLength) < 0)
  {
    outWarn("could not get socket receive buffer size.");
  }
  outInfo("socket receive buffer size is: " << receiveBufferSize);
}

void UnrealVisionBridge::receive()
{
  const size_t minSize = std::min((size_t)1024, bufferActive.size());
  uint8_t *pPackage = &bufferActive[0];
  PacketHeader header;
  memset(&header, 0, sizeof(PacketHeader));
  size_t written = 0, left = minSize;

  outInfo("receiver started.");
  while(ros::ok() && running)
  {
    if(!isConnected)
    {
      connectToServer();
      continue;
    }

    ssize_t bytesRead = read(connection, pPackage + written, left);
    if(bytesRead <= 0)
    {
      outError("could not read from socket.");
      close(connection);
      isConnected = false;
      continue;
    }

    left -= bytesRead;
    written += bytesRead;

    if(header.size == 0 && written > sizeof(PacketHeader))
    {
      header = *reinterpret_cast<PacketHeader *>(pPackage);

      if(bufferActive.size() < header.size)
      {
        // make it 1 mb bigger that the actual package size, so that the buffer does not need to be resized often
        bufferActive.resize(header.size + 1024 * 1024);
        pPackage = &bufferActive[0];
        outInfo("resized buffer to: " << bufferActive.size());
      }
      left = header.size - written;
    }

    if(header.size != 0 && left == 0)
    {
      uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      outDebug("package complete. delay: " << (now - header.timestampSent) / 1000000.0 << " ms.");

      if(header.sizeHeader != sizeof(PacketHeader))
      {
        outError("package header size does not match expectations: " << sizeof(PacketHeader) << " received: " << header.sizeHeader);
        close(connection);
        isConnected = false;
        continue;
      }

      lockBuffer.lock();
      bufferActive.swap(bufferComplete);

      packet.header = header;
      packet.sizeColor = header.width * header.height * sizeRGB;
      packet.sizeDepth = header.width * header.height * sizeFloat;
      packet.sizeObject = header.width * header.height * sizeRGB;
      packet.pColor = &bufferComplete[sizeof(PacketHeader)];
      packet.pDepth = packet.pColor + packet.sizeColor;
      packet.pObject = packet.pDepth + packet.sizeDepth;
      packet.pMap = packet.pObject + packet.sizeColor;
      _newData = true;

      pPackage = &bufferActive[0];
      header.size = 0;
      written = 0;
      left = minSize;
      lockBuffer.unlock();
    }
  }
  close(connection);
  isConnected = false;
  running = false;
  ros::shutdown();
  outInfo("receiver stopped.");
}

bool UnrealVisionBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData())
  {
    return false;
  }
  MEASURE_TIME;

  lockBuffer.lock();
  bufferComplete.swap(bufferInUse);
  Packet packet = this->packet;
  _newData = false;
  lockBuffer.unlock();

  // set transform and timestamp
  uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  ros::Time stamp;
  stamp.fromNSec((ros::Time::now() - ros::Time().fromNSec(now - packet.header.timestampCapture)).toNSec());

  rs::SceneCas cas(tcas);
  rs::Scene scene = cas.getScene();
  tf::Vector3 translation(packet.header.translation.x, packet.header.translation.y, packet.header.translation.z);
  tf::Quaternion rotation(packet.header.rotation.x, packet.header.rotation.y, packet.header.rotation.z, packet.header.rotation.w);
  tf::Quaternion rotationCamera;
  rotationCamera.setEuler(90.0 * M_PI / 180.0, 0.0, -90.0 * M_PI / 180.0);
  rotation = rotation * rotationCamera;

  rs::StampedTransform vp(rs::conversion::to(tcas, tf::StampedTransform(tf::Transform(rotation, translation), stamp, tfTo, tfFrom)));
  scene.viewPoint.set(vp);
  scene.timestamp.set(stamp.toNSec());

  // setting images
  cv::Mat color(packet.header.height, packet.header.width, CV_8UC3, packet.pColor);
  cv::Mat depth(packet.header.height, packet.header.width, CV_32FC1);
  cv::Mat object(packet.header.height, packet.header.width, CV_8UC3, packet.pObject);

  // converting depth data
  convertDepth(reinterpret_cast<uint16_t *>(packet.pDepth), depth.ptr<__m128>());

  // getting object color map
  std::map<std::string, cv::Vec3b> objectMap;
  const size_t SizeEntryHeader = sizeof(uint32_t) + 3 * sizeof(uint8_t);
  uint8_t *it = packet.pMap;
  for(uint32_t i = 0; i < packet.header.mapEntries; ++i)
  {
    const MapEntry *entry = reinterpret_cast<MapEntry *>(it);
    cv::Vec3b color;
    color.val[0] = entry->b;
    color.val[1] = entry->g;
    color.val[2] = entry->r;
    objectMap[std::string(&entry->firstChar, entry->size - SizeEntryHeader)] = color;
    it += entry->size;
  }

  // setting camera info
  sensor_msgs::CameraInfo cameraInfo;
  const double halfFOVX = packet.header.fieldOfViewX * M_PI / 360.0;
  const double halfFOVY = packet.header.fieldOfViewY * M_PI / 360.0;
  const double cX = packet.header.width / 2.0;
  const double cY = packet.header.height / 2.0;

  cameraInfo.header.frame_id = tfFrom;
  cameraInfo.header.stamp = stamp;
  cameraInfo.height = packet.header.height;
  cameraInfo.width = packet.header.width;

  cameraInfo.K.assign(0.0);
  cameraInfo.K[0] = cX / std::tan(halfFOVX);
  cameraInfo.K[2] = cX;
  cameraInfo.K[4] = cY / std::tan(halfFOVY);
  cameraInfo.K[5] = cY;
  cameraInfo.K[8] = 1;

  cameraInfo.R.assign(0.0);
  cameraInfo.R[0] = 1;
  cameraInfo.R[4] = 1;
  cameraInfo.R[8] = 1;

  cameraInfo.P.assign(0.0);
  cameraInfo.P[0] = cameraInfo.K[0];
  cameraInfo.P[2] = cameraInfo.K[2];
  cameraInfo.P[5] = cameraInfo.K[4];
  cameraInfo.P[6] = cameraInfo.K[5];
  cameraInfo.P[10] = 1;

  cameraInfo.distortion_model = "plumb_bob";
  cameraInfo.D.resize(5, 0.0);

  // setting cas
  cas.set(VIEW_CAMERA_INFO, cameraInfo);
  cas.set(VIEW_COLOR_IMAGE, color);
  cas.set(VIEW_DEPTH_IMAGE, depth);
  cas.set(VIEW_OBJECT_IMAGE, object);
  cas.set(VIEW_OBJECT_MAP, objectMap);

  return true;
}
