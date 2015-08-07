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

#include "ImageMultiplexer.h"

#include <algorithm>

// Define the aspect ratio of the image grid
#define IMAGE_ASPECT_RATIO  (4.0f / 3.0f)

ImageMultiplexer::ImageMultiplexer(unsigned int width, unsigned int height)
  : m_width(width),
    m_height(height),
    m_image(cv::Mat::zeros(height, width, CV_8UC3) + 0xff) // blue (BGR8)
{
}

void ImageMultiplexer::RegisterComputer(const std::string& strComputer)
{
  m_computers.insert(strComputer);
}

void ImageMultiplexer::UnregisterComputer(const std::string& strComputer)
{
  std::set<std::string>::iterator it = m_computers.find(strComputer);
  if (it != m_computers.end())
    m_computers.erase(it);
}

void ImageMultiplexer::AddImage(const std::string& strComputer, ImageTopic topic, const cv::Mat& image)
{
  unsigned int width;
  unsigned int height;
  GetImageDimensions(width, height);

  cv::Mat  resizedImage = ResizeImage(image, width, height);

  cv::Point point1;
  cv::Point point2;
  if (GetDestination(strComputer, topic, point1, point2))
  {
    // TODO: Copy resizedImage to m_image from point1 to point2
  }
}

void ImageMultiplexer::GetImageCount(unsigned int& cols, unsigned int& rows) const
{
  cols = m_computers.size();
  rows = static_cast<unsigned int>(IMAGE_TOPIC_COUNT);
}

void ImageMultiplexer::GetImageDimensions(unsigned int& width, unsigned int& height) const
{
  unsigned int cols;
  unsigned int rows;
  GetImageCount(cols, rows);

  const float maxWidth = (float)m_width / (float)cols;
  const float maxHeight = (float)m_height / (float)rows;

  width = (unsigned int)std::min(maxWidth, maxHeight * IMAGE_ASPECT_RATIO);
  height = (unsigned int)std::min(maxHeight, maxWidth / IMAGE_ASPECT_RATIO);
}

cv::Mat ImageMultiplexer::ResizeImage(const cv::Mat& image, unsigned int width, unsigned int height) const
{
  cv::Mat resizedImage;

  // TODO

  return resizedImage;
}

int ImageMultiplexer::GetIndex(const std::string& strComputer) const
{
  unsigned int i = 0;
  for (std::set<std::string>::const_iterator it = m_computers.begin(); it != m_computers.end(); ++it)
  {
    if (*it == strComputer)
      return i;
    i++;
  }

  return -1;
}

int ImageMultiplexer::GetIndex(ImageTopic topic) const
{
  int index = static_cast<int>(topic);
  if (0 <= index && index < static_cast<int>(IMAGE_TOPIC_COUNT))
    return index;

  return -1;
}

bool ImageMultiplexer::GetDestination(const std::string& strComputer,
                                      ImageTopic topic,
                                      cv::Point& point1,
                                      cv::Point& point2) const
{

  int col = GetIndex(strComputer);
  int row = GetIndex(topic);

  if (col >= 0 && row >= 0)
  {
    unsigned int width;
    unsigned int height;
    GetImageDimensions(width, height);

    point1.x = col * width;
    point1.y = row * height;

    point2.x = point1.x + width - 1;
    point2.y = point1.y + height - 1;

    return true;
  }

  return false;
}
