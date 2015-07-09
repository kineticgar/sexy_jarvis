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

#include <ros/ros.h>

#define NODE_NAME     "image_multiplexer"
#define VIDEO_WIDTH   1920
#define VIDEO_HEIGHT  1080
#define VIDEO_FPS     1 // TODO: Change to 30

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);

  ImageMultiplexerNode node(VIDEO_WIDTH, VIDEO_HEIGHT);
  if (node.Initialize())
  {
    ros::Rate loop_rate(VIDEO_FPS);
    while (node.OK())
    {
      node.PublishImage();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}
