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

#include "ImageSubscriber.h"

#include "image_transport/transport_hints.h"

#include <assert.h>

ImageSubscriber::ImageSubscriber(const std::string& strComputer,
                                 ImageTopic topic,
                                 ImageTransport transport,
                                 image_transport::ImageTransport& imageTransport,
                                 ImageSubscriberCallback* callback) :
  m_strComputer(strComputer),
  m_topic(topic),
  m_callback(callback)
{
  assert(m_callback);

  m_subscriber = imageTransport.subscribe(
      strComputer + "/" + TranslateTopic(topic),
      1,
      &ImageSubscriber::ReceiveImage,
      this,
      image_transport::TransportHints(TranslateTransport(transport))
  );
}

void ImageSubscriber::ReceiveImage(const sensor_msgs::ImageConstPtr& msg)
{
  m_callback->ReceiveImage(m_strComputer, m_topic, msg);
}

const char* ImageSubscriber::TranslateTopic(ImageTopic topic)
{
  switch (topic)
  {
    case IMAGE_TOPIC_IMAGE_RAW:  return "image_raw";
    case IMAGE_TOPIC_FOREGROUND: return "foreground";
    case IMAGE_TOPIC_BACKGROUND: return "background";
    default:
      break;
  }
  return "";
}

const char* ImageSubscriber::TranslateTransport(ImageTransport transport)
{
  switch (transport)
  {
    case IMAGE_TRANSPORT_RAW:              return "raw";
    case IMAGE_TRANSPORT_COMPRESSED:       return "compressed";
    case IMAGE_TRANSPORT_COMPRESSED_DEPTH: return "compressedDepth";
    case IMAGE_TRANSPORT_THEORA:           return "theora";
    default:
      break;
  }
  return "";
}
