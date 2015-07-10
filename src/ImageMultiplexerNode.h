/*
 *      Copyright (c) 2015 Garrett Brown
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 *  deal in the Software without restriction, including without limitation the
 *  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 *  sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *      The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 *  IN THE SOFTWARE.
 */
#pragma once

#include "ImageMultiplexer.h"
#include "ImageSubscriber.h"

#include "image_transport/image_transport.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "threads/mutex.h"

#include <map>
#include <memory>
#include <string>

class ImageMultiplexerNode : public ImageSubscriberCallback
{
public:
  ImageMultiplexerNode(unsigned int width, unsigned int height);
  ~ImageMultiplexerNode(void) { Deinitialize(); }

  bool Initialize(void);
  void Deinitialize(void);

  bool OK(void) const { return m_node.ok(); }

  virtual void ReceiveImage(const std::string& strComputer,
                            ImageTopic topic,
                            const sensor_msgs::ImageConstPtr& msg);

  void PublishImage(void);

private:
  std::vector<std::string> GetMachineNames(void);

  ImageSubscriberPtr GetImageSubscriber(const std::string& strComputer,
                                        ImageTopic topic,
                                        ImageTransport transport);

  typedef std::map<ImageTopic, ImageSubscriberPtr> TopicMap;
  typedef std::string                              ComputerName;
  typedef std::map<ComputerName, TopicMap>         SubscriberMap;

  ros::NodeHandle                 m_node;
  image_transport::ImageTransport m_transport;
  image_transport::Publisher      m_publisher;
  SubscriberMap                   m_subscribers;
  ImageMultiplexer                m_multiplexer;
  PLATFORM::CMutex                m_multiplexerMutex;
};
