/**Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       main.cpp
 * @date       11/23/2019
 * @brief      The project will explore and generate map of one unknown environment and
 *             also detect tags and display tags information in rviz
 * @license    This project is released under the BSD-3-Clause License.
 *             Redistribution and use in source and binary forms, with or without
 *             modification, are permitted provided that the following conditions are met:
 *
 *             1. Redistributions of source code must retain the above copyright notice, this
 *                list of conditions and the following disclaimer.
 *
 *             2. Redistributions in binary form must reproduce the above copyright notice,
 *                this list of conditions and the following disclaimer in the documentation
 *                and/or other materials provided with the distribution.
 *
 *             3. Neither the name of the copyright holder nor the names of its
 *                contributors may be used to endorse or promote products derived from
 *                this software without specific prior written permission.
 *
 *             THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *             AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *             IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *             DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *             FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *             DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *             SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *             CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *             OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *             OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <terprescue.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <qrcode.hpp>

using namespace std;
using namespace cv;

void display(Mat &im, Mat &bbox)
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  }
  imshow("Result", im);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "terprescue");

  ros::NodeHandle nh;
  // TerpRescue terpRescue;
  ros::Rate loop_rate(10);
  // terpRescue.detectTags();
  // terpRescue.visualization();

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : " << CV_MAJOR_VERSION << endl;
  cout << "Minor version : " << CV_MINOR_VERSION << endl;
  cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
  cout << "Initializing QRCodeDetector:\n";
  cout << "\nInitialize finished!\n";
  if ( CV_MAJOR_VERSION < 3)
  {
      // Old OpenCV 2 code goes here.
  } else
  {
      // New OpenCV 3 code goes here.
  }

  // Read image
  Mat inputImage;
  if(argc>1)
    inputImage = imread(argv[1]);
  else{
    inputImage = imread("/home/zycao/catkin_ws/src/terprescue/img/qr_in_web.png");
    // inputImage = imread("./img/hello_world.png");
    imshow("Input", inputImage);
    waitKey(0);
  }
  QRCodeDetector qrDecoder;

  Mat bbox, rectifiedImage;

  std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
  if(data.length()>0)
  {
    cout << "Decoded Data : " << data << endl;

    display(inputImage, bbox);
    // rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
    // imshow("Rectified QRCode", rectifiedImage);

    waitKey(0);
  }
  else
    cout << "QR Code not detected" << endl;



    ros::spin();

    return 0;
}
