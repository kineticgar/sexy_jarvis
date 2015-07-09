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

#include "ImageMultiplexerNode.h"

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "XmlRpcValue.h"

using namespace PLATFORM;
using namespace XmlRpc;

ImageMultiplexerNode::ImageMultiplexerNode(unsigned int width, unsigned int height) :
  m_multiplexer(width, height),
  m_node(),
  m_transport(m_node)
{
}

bool ImageMultiplexerNode::Initialize(void)
{
  std::vector<std::string> computers = GetMachineNames();
  for (std::vector<std::string>::const_iterator it = computers.begin(); it != computers.end(); ++it)
  {
    const std::string& strComputer = *it;
    TopicMap& topics = m_subscribers[strComputer];

    topics[IMAGE_TOPIC_IMAGE_RAW]  = GetImageSubscriber(strComputer, IMAGE_TOPIC_IMAGE_RAW,  IMAGE_TRANSPORT_COMPRESSED);
    topics[IMAGE_TOPIC_FOREGROUND] = GetImageSubscriber(strComputer, IMAGE_TOPIC_FOREGROUND, IMAGE_TRANSPORT_RAW);
    topics[IMAGE_TOPIC_BACKGROUND] = GetImageSubscriber(strComputer, IMAGE_TOPIC_BACKGROUND, IMAGE_TRANSPORT_RAW);
  }

   m_publisher = m_transport.advertise("video", 1);

  return true;
}

void ImageMultiplexerNode::Deinitialize(void)
{
  m_subscribers.clear();
}

void ImageMultiplexerNode::ReceiveImage(const std::string& strComputer, ImageTopic topic, const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr)
  {
    CLockObject lock(m_mutex);
    m_multiplexer.AddImage(strComputer, topic, cv_ptr->image);
  }
}

void ImageMultiplexerNode::PublishImage(void)
{
  CLockObject lock(m_mutex);
  cv_bridge::CvImage image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_multiplexer.GetImage());
  m_publisher.publish(image.toImageMsg());
}

std::vector<std::string> ImageMultiplexerNode::GetMachineNames(void)
{
  std::vector<std::string> names;

  XmlRpcValue machines;
  if (m_node.getParam("machines", machines) && machines.getType() == XmlRpcValue::TypeStruct)
  {
    for (XmlRpcValue::ValueStruct::const_iterator it = machines.begin(); it != machines.end(); ++it)
    {
      const std::string& strComputer = it->first;
      const XmlRpcValue& properties = it->second;

      if (properties.getType() == XmlRpcValue::TypeStruct)
      {
        // Only get machines with cameras attached
        bool bHasCamera = false;
        for (XmlRpcValue::ValueStruct::const_iterator it2 = const_cast<XmlRpcValue&>(properties).begin();
            it2 != const_cast<XmlRpcValue&>(properties).end();
            ++it2)
        {
          const std::string& strProperty = it2->first;
          const XmlRpcValue& value = it->second;
          if (strProperty == "camera" && value.getType() == XmlRpcValue::TypeString)
          {
            bHasCamera = true;
            break;
          }
        }
        if (bHasCamera)
          names.push_back(it->first);
      }
    }
  }

  return names;
}

ImageSubscriberPtr ImageMultiplexerNode::GetImageSubscriber(const std::string& strComputer,
                                                            ImageTopic topic,
                                                            ImageTransport transport)
{
  return ImageSubscriberPtr(new ImageSubscriber(strComputer, topic, transport, m_transport, this));
}
